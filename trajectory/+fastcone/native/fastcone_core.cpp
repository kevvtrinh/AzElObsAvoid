// MATLAB MEX implementation of the scalar and 3D Lorentz equations in
// +fastcone/solveNative.m. Eigen supplies sparse matrix algebra and Cholesky only.
// This is not an external optimization solver. Every accepted point is
// checked against the original unscaled problem and a separate weak bound.
#include "mex.h"
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using Vec=Eigen::VectorXd;
using Sp=Eigen::SparseMatrix<double,Eigen::ColMajor,int>;
using Trip=Eigen::Triplet<double>;
constexpr double tiny=std::numeric_limits<double>::min();
constexpr double eps=std::numeric_limits<double>::epsilon();
constexpr double inf=std::numeric_limits<double>::infinity();
static void need(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
static double normInf(const Vec& v) { return v.size()?v.cwiseAbs().maxCoeff():0.0; }
static const mxArray* field(const mxArray* a,const char* name) {
    need(mxIsStruct(a)&&mxGetNumberOfElements(a)==1,"Validation must be a scalar struct.");
    const mxArray* v=mxGetField(a,0,name); need(v!=nullptr,"Missing validation field."); return v;
}
static double scalar(const mxArray* a) {
    need(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a)&&mxGetNumberOfElements(a)==1,"Expected real double scalar.");
    return mxGetScalar(a);
}
static Vec vectorFrom(const mxArray* a) {
    need(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a),"Expected dense real double vector.");
    need(mxGetM(a)<=1||mxGetN(a)<=1,"Expected a vector.");
    const mwSize size=mxGetNumberOfElements(a); need(size<=INT_MAX,"Vector exceeds supported dimensions.");
    if(!size) return Vec(0);
    return Eigen::Map<const Vec>(mxGetPr(a),static_cast<int>(size));
}
static Sp sparseFrom(const mxArray* a) {
    need(mxIsDouble(a)&&!mxIsComplex(a)&&mxIsSparse(a),"Expected real sparse double matrix.");
    const mwSize m=mxGetM(a),n=mxGetN(a); const mwIndex* col=mxGetJc(a); const mwIndex* row=mxGetIr(a);
    need(m<=INT_MAX&&n<=INT_MAX&&col[n]<=INT_MAX,"Sparse matrix exceeds supported dimensions.");
    const int nz=static_cast<int>(col[n]); std::vector<int> outer(n+1),inner(nz);
    for(mwSize j=0;j<=n;++j) outer[j]=static_cast<int>(col[j]);
    for(int p=0;p<nz;++p) inner[p]=static_cast<int>(row[p]);
    return Eigen::Map<const Sp>(static_cast<int>(m),static_cast<int>(n),nz,outer.data(),inner.data(),mxGetPr(a));
}
static mxArray* toMx(const Vec& v) {
    mxArray* out=mxCreateDoubleMatrix(v.size(),1,mxREAL);
    std::copy(v.data(),v.data()+v.size(),mxGetPr(out)); return out;
}

// Prepare the fixed lower-triangle products once. Iterations change only W.
struct GramTerm {int row,col,weight,destination;double coefficient;};
struct PreparedGram {
    Sp pattern;std::vector<GramTerm> terms;
    PreparedGram(const Sp& gt,int nl,int nc) {
        int n=gt.rows();std::vector<Trip> structural;
        auto add=[&](int r,int s,int weight) {
            for(Sp::InnerIterator a(gt,r);a;++a) for(Sp::InnerIterator b(gt,s);b;++b) {
                if(a.row()<b.row()) continue;
                double product=a.value()*b.value();
                if(product==0) continue;
                terms.push_back({static_cast<int>(a.row()),static_cast<int>(b.row()),weight,0,product});
                structural.emplace_back(a.row(),b.row(),1.0);
            }
        };
        for(int i=0;i<nl;++i) add(i,i,i);
        for(int k=0;k<nc;++k) for(int r=0;r<3;++r) for(int s=0;s<3;++s)
            add(nl+3*k+r,nl+3*k+s,nl+9*k+3*s+r);
        for(int j=0;j<n;++j) structural.emplace_back(j,j,1.0);
        pattern.resize(n,n);pattern.setFromTriplets(structural.begin(),structural.end());
        pattern.makeCompressed();
        for(auto& term:terms) {
            const int* begin=pattern.innerIndexPtr()+pattern.outerIndexPtr()[term.col];
            const int* end=pattern.innerIndexPtr()+pattern.outerIndexPtr()[term.col+1];
            const int* found=std::lower_bound(begin,end,term.row);
            need(found!=end&&*found==term.row,"Missing prepared Gram slot.");
            term.destination=static_cast<int>(found-pattern.innerIndexPtr());
        }
    }
    Sp evaluate(const Sp& w) const {
        Sp k=pattern;std::fill(k.valuePtr(),k.valuePtr()+k.nonZeros(),0.0);
        for(const auto& term:terms) k.valuePtr()[term.destination]+=term.coefficient*w.valuePtr()[term.weight];
        return k;
    }
};
struct Validation {
    Sp A,E,map,coneG; Vec b,d,lower,upper,base,coneH;
    explicit Validation(const mxArray* a,int n) {
        A=sparseFrom(field(a,"A")); E=sparseFrom(field(a,"E")); map=sparseFrom(field(a,"Map"));
        coneG=sparseFrom(field(a,"ConeG")); b=vectorFrom(field(a,"B")); d=vectorFrom(field(a,"D"));
        lower=vectorFrom(field(a,"Lower")); upper=vectorFrom(field(a,"Upper"));
        base=vectorFrom(field(a,"Base")); coneH=vectorFrom(field(a,"ConeH"));
        const int physical=base.size();
        need(map.rows()==physical&&map.cols()==n&&A.cols()==physical&&E.cols()==physical&&coneG.cols()==physical,"Validation matrix dimensions disagree.");
        need(A.rows()==b.size()&&E.rows()==d.size()&&lower.size()==physical&&upper.size()==physical&&coneG.rows()==coneH.size()&&coneH.size()%3==0,"Validation vector dimensions disagree.");
    }
    double residual(const Vec& q) const {
        Vec x=base+map*q; if(!x.allFinite()) return inf;
        double value=0;
        if(A.rows()) value=std::max(value,(A*x-b).maxCoeff());
        if(E.rows()) value=std::max(value,normInf(E*x-d));
        for(int j=0;j<x.size();++j) value=std::max({value,lower[j]-x[j],x[j]-upper[j]});
        Vec s=coneH-coneG*x;
        for(int j=0;j<s.size();j+=3) value=std::max(value,std::hypot(s[j+1],s[j+2])-s[j]);
        return value;
    }
};

static Vec jordan(const Vec& a,const Vec& b,int nl) {
    Vec out(a.size());
    for(int i=0;i<nl;++i) out[i]=a[i]*b[i];
    for(int i=nl;i<a.size();i+=3) {
        out[i]=a[i]*b[i]+a[i+1]*b[i+1]+a[i+2]*b[i+2];
        out[i+1]=a[i]*b[i+1]+b[i]*a[i+1]; out[i+2]=a[i]*b[i+2]+b[i]*a[i+2];
    }
    return out;
}
static Vec jordanInverse(const Vec& a,const Vec& b,int nl) {
    Vec out(a.size());
    for(int i=0;i<nl;++i) out[i]=b[i]/a[i];
    for(int i=nl;i<a.size();i+=3) {
        out[i]=(a[i]*b[i]-a[i+1]*b[i+1]-a[i+2]*b[i+2])/(a[i]*a[i]-a[i+1]*a[i+1]-a[i+2]*a[i+2]);
        out[i+1]=(b[i+1]-a[i+1]*out[i])/a[i]; out[i+2]=(b[i+2]-a[i+2]*out[i])/a[i];
    }
    return out;
}
static double boundary(const Vec& a,const Vec& b,int nl) {
    double alpha=1;
    for(int i=0;i<nl;++i) if(b[i]<0) alpha=std::min(alpha,-a[i]/b[i]);
    auto take=[&](double root) { if(std::isfinite(root)&&root>0) alpha=std::min(alpha,root); };
    for(int i=nl;i<a.size();i+=3) {
        if(a[i]+alpha*b[i]>=std::hypot(a[i+1]+alpha*b[i+1],a[i+2]+alpha*b[i+2])) continue;
        double qa=b[i]*b[i]-b[i+1]*b[i+1]-b[i+2]*b[i+2];
        double qb=2*(a[i]*b[i]-a[i+1]*b[i+1]-a[i+2]*b[i+2]);
        double qc=a[i]*a[i]-a[i+1]*a[i+1]-a[i+2]*a[i+2];
        if(std::abs(qa)<eps*std::max(1.0,std::abs(qb))) take(-qc/qb);
        else {
            double q=-.5*(qb+std::copysign(1.0,qb==0?1.0:qb)*std::sqrt(std::max(0.0,qb*qb-4*qa*qc)));
            take(q/qa); take(qc/q);
        }
        if(b[i]<0) take(-a[i]/b[i]);
    }
    return std::max(0.0,alpha);
}

static void scaling(const Vec& s,const Vec& z,int nl,Sp& W,Sp& Di,Sp& D,Vec& v) {
    const int m=s.size();
    // The cone graph is fixed. Allocate each scalar/3x3 block only once.
    if(W.rows()!=m) {
        std::vector<Trip> entries;entries.reserve(nl+3*(m-nl));
        for(int i=0;i<nl;++i) entries.emplace_back(i,i,1.0);
        for(int i=nl;i<m;i+=3) for(int col=0;col<3;++col) for(int row=0;row<3;++row)
            entries.emplace_back(i+row,i+col,1.0);
        W.resize(m,m);W.setFromTriplets(entries.begin(),entries.end());Di=W;D=W;
    }
    for(int i=0;i<nl;++i) {
        double r=z[i]/s[i],root=std::sqrt(r);W.valuePtr()[i]=r;Di.valuePtr()[i]=root;D.valuePtr()[i]=1/root;
    }
    for(int i=nl;i<m;i+=3) {
        double da=std::sqrt(std::max(tiny,s[i]*s[i]-s[i+1]*s[i+1]-s[i+2]*s[i+2]));
        double db=std::sqrt(std::max(tiny,z[i]*z[i]-z[i+1]*z[i+1]-z[i+2]*z[i+2]));
        double a0=s[i]/da,a1=s[i+1]/da,a2=s[i+2]/da,b0=z[i]/db,b1=z[i+1]/db,b2=z[i+2]/db;
        double multiplier=std::sqrt(da/db)/std::sqrt(2*(1+a0*b0+a1*b1+a2*b2));
        double wt=(a0+b0)*multiplier,wx=(a1-b1)*multiplier,wy=(a2-b2)*multiplier;
        double det=da/db,root=std::sqrt(det),denom=det*(wt+root),dw=det*det;
        double inverse[9]={wt/det,-wx/det,-wy/det,-wx/det,1/root+wx*wx/denom,wx*wy/denom,-wy/det,wx*wy/denom,1/root+wy*wy/denom};
        double direct[9]={wt,wx,wy,wx,root+wx*wx/(wt+root),wx*wy/(wt+root),wy,wx*wy/(wt+root),root+wy*wy/(wt+root)};
        double weight[9]={(wt*wt+wx*wx+wy*wy)/dw,-2*wt*wx/dw,-2*wt*wy/dw,-2*wt*wx/dw,1/det+2*wx*wx/dw,2*wx*wy/dw,-2*wt*wy/dw,2*wx*wy/dw,1/det+2*wy*wy/dw};
        for(int col=0;col<3;++col) for(int row=0;row<3;++row) {
            int at=3*col+row,slot=nl+3*(i-nl)+at;W.valuePtr()[slot]=weight[at];Di.valuePtr()[slot]=inverse[at];D.valuePtr()[slot]=direct[at];
        }
    }
    v=Di*s;
}

static double weakBound(const Vec& cost,const Sp& GT,const Vec& h,Vec z,const Vec& lb,const Vec& ub,int nl) {
    for(int i=0;i<nl;++i) z[i]=std::max(0.0,z[i]);
    for(int i=nl;i<z.size();i+=3) {
        double radius=std::hypot(z[i+1],z[i+2]);
        if(z[i]<=-radius) z.segment<3>(i).setZero();
        else if(z[i]<radius) {
            double head=.5*(radius+z[i]);z[i+1]*=head/radius;z[i+2]*=head/radius;
            double x=std::max(1.0,head),ulp=std::nextafter(x,inf)-x;
            z[i]=std::max(head,std::hypot(z[i+1],z[i+2]))+4*ulp;
        }
    }
    Vec product=GT*z; double scale=1; bool hasInfinite=false;
    for(int i=0;i<cost.size();++i) {
        if(ub[i]==inf&&product[i]<0) {scale=std::min(scale,cost[i]/(-product[i]));hasInfinite=true;}
        if(lb[i]==-inf&&product[i]>0) {scale=std::min(scale,-cost[i]/product[i]);hasInfinite=true;}
    }
    scale=std::max(0.0,std::min(1.0,scale)); if(scale>0&&(scale<1||hasInfinite)) scale*=1-8*eps;
    Vec residual=cost+scale*product;double bound=-scale*h.dot(z);
    for(int i=0;i<cost.size();++i) if(residual[i]!=0) {
        double endpoint=residual[i]<0?ub[i]:lb[i]; if(!std::isfinite(endpoint)) return -inf;
        bound+=residual[i]*endpoint;
    }
    return std::isfinite(bound)?bound:-inf;
}

extern "C" void mexFunction(int nlhs,mxArray* plhs[],int nrhs,const mxArray* prhs[]) {
    try {
        need(nrhs==13&&nlhs==3,"Expected 13 inputs and three outputs.");
        auto start=std::chrono::steady_clock::now();
        Sp G=sparseFrom(prhs[0]),GT=G.transpose();Vec h=vectorFrom(prhs[1]),cost=vectorFrom(prhs[2]),lb=vectorFrom(prhs[3]),ub=vectorFrom(prhs[4]);
        double nlValue=scalar(prhs[5]),ncValue=scalar(prhs[6]),tol=scalar(prhs[7]),optTol=scalar(prhs[8]),maxValue=scalar(prhs[9]);
        double constant=scalar(prhs[10]),costScale=scalar(prhs[11]);
        need(std::isfinite(nlValue)&&nlValue>=0&&nlValue==std::floor(nlValue)&&nlValue<=INT_MAX,"Invalid scalar cone count.");
        need(std::isfinite(ncValue)&&ncValue>=0&&ncValue==std::floor(ncValue)&&ncValue<=INT_MAX/3,"Invalid Lorentz cone count.");
        need(std::isfinite(maxValue)&&maxValue>=1&&maxValue<=100000&&maxValue==std::floor(maxValue),"Invalid iteration limit.");
        need(std::isfinite(tol)&&tol>0&&std::isfinite(optTol)&&optTol>0&&std::isfinite(costScale)&&costScale>0&&std::isfinite(constant),"Invalid scalar solver data.");
        int nl=static_cast<int>(nlValue),nc=static_cast<int>(ncValue),maxIter=static_cast<int>(maxValue),m=G.rows(),n=G.cols();
        need(n>0&&m==nl+3*nc&&h.size()==m&&cost.size()==n&&lb.size()==n&&ub.size()==n,"Reduced dimensions disagree.");
        need(h.allFinite()&&cost.allFinite(),"Nonfinite reduced data.");
        for(int p=0;p<G.nonZeros();++p) need(std::isfinite(G.valuePtr()[p]),"Nonfinite sparse matrix value.");
        Validation check(prhs[12],n); PreparedGram prepared(GT,nl,nc);
        Vec identity=Vec::Zero(m);identity.head(nl).setOnes();for(int i=nl;i<m;i+=3) identity[i]=1;
        Vec s=identity,z=identity,x=Vec::Zero(n),rg,rd,v;Sp W,Di,D;
        Eigen::SimplicialLLT<Sp,Eigen::Lower,Eigen::AMDOrdering<int>> factor;
        std::vector<int> previousOuter,previousInner;int analyses=0,regularized=0,iteration=0,flag=0;
        double primal=inf,dual=inf,gap=inf,lowerBound=-inf,objectiveGap=inf,originalResidual=inf;
        for(int it=1;it<=maxIter;++it) {
            iteration=it;rg=G*x+s-h;rd=cost+GT*z;gap=s.dot(z);
            primal=normInf(rg)/(1+normInf(h));dual=normInf(rd)/(1+normInf(cost));
            double relativeGap=gap/(1+std::abs(cost.dot(x)));
            if(primal<=tol*.1&&relativeGap<=std::sqrt(optTol)) {
                lowerBound=constant+costScale*weakBound(cost,GT,h,z,lb,ub,nl);
                double value=constant+costScale*cost.dot(x);objectiveGap=value-lowerBound;
                if(std::abs(objectiveGap)<=optTol*(1+std::abs(value))) {
                    originalResidual=check.residual(x);
                    if(originalResidual<=tol) {flag=1;break;}
                }
            }
            double mu=gap/std::max(1,nl+nc);
            scaling(s,z,nl,W,Di,D,v);
            Sp K=prepared.evaluate(W);
            Vec equilibration(n);for(int i=0;i<n;++i) equilibration[i]=1/std::sqrt(std::max(tiny,K.coeff(i,i)));
            for(int col=0;col<n;++col) for(Sp::InnerIterator entry(K,col);entry;++entry) entry.valueRef()*=equilibration[entry.row()]*equilibration[col];
            for(int i=0;i<n;++i) K.coeffRef(i,i)+=0;
            K.makeCompressed();bool finite=true;for(int p=0;p<K.nonZeros();++p) if(!std::isfinite(K.valuePtr()[p])) finite=false;
            if(!finite) break;
            bool same=previousOuter.size()==static_cast<size_t>(n+1)&&previousInner.size()==static_cast<size_t>(K.nonZeros());
            if(same) same=std::equal(previousOuter.begin(),previousOuter.end(),K.outerIndexPtr())&&std::equal(previousInner.begin(),previousInner.end(),K.innerIndexPtr());
            if(!same) {
                factor.analyzePattern(K);++analyses;
                previousOuter.assign(K.outerIndexPtr(),K.outerIndexPtr()+n+1);previousInner.assign(K.innerIndexPtr(),K.innerIndexPtr()+K.nonZeros());
            }
            bool factored=false;double previousReg=0;
            for(double reg:{0.0,1e-14,1e-12,1e-10,1e-8}) {
                for(int i=0;i<n;++i) K.coeffRef(i,i)+=reg-previousReg;
                previousReg=reg;factor.factorize(K);
                if(factor.info()==Eigen::Success) {factored=true;if(reg>0) ++regularized;break;}
            }
            if(!factored) break;
            auto direction=[&](const Vec& rc,Vec& dx,Vec& ds,Vec& dz) {
                Vec rhsCone=Di*jordanInverse(v,-rc,nl)+W*rg;
                Vec rhs=equilibration.array()*(-rd-GT*rhsCone).array();
                dx=equilibration.array()*factor.solve(rhs).array();Vec gx=G*dx;
                ds=-rg-gx;dz=rhsCone+W*gx;
            };
            Vec rc=jordan(v,v,nl),dx,ds,dz;direction(rc,dx,ds,dz);
            if(!dx.allFinite()||!ds.allFinite()||!dz.allFinite()) break;
            double ap=boundary(s,ds,nl),ad=boundary(z,dz,nl);
            double muAffine=(s+ap*ds).dot(z+ad*dz)/std::max(1,nl+nc);
            double sigma=std::clamp(std::pow(std::max(0.0,muAffine/mu),3),0.0,1.0);
            rc+=jordan(Di*ds,D*dz,nl)-sigma*mu*identity;direction(rc,dx,ds,dz);
            ap=std::min(1.0,.995*boundary(s,ds,nl));ad=std::min(1.0,.995*boundary(z,dz,nl));
            if(!dx.allFinite()||!ds.allFinite()||!dz.allFinite()||std::min(ap,ad)<1e-14) break;
            x+=ap*dx;s+=ap*ds;z+=ad*dz;
        }
        if(flag!=1) originalResidual=check.residual(x);
        double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        Vec stats(13);stats<<iteration,flag,primal,dual,gap,lowerBound,objectiveGap,originalResidual,analyses,regularized,wall,n,G.nonZeros();
        plhs[0]=toMx(x);plhs[1]=toMx(z);plhs[2]=toMx(stats);
    } catch(const std::exception& error) { mexErrMsgIdAndTxt("fastcone:nativeCore","%s",error.what()); }
}
