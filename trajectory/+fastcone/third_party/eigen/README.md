# Eigen 3.4.0

Unmodified Eigen headers from tag `3.4.0`, commit
`3147391d946bb4b6c68edd901f2add6ac1f31f8c` of
https://gitlab.com/libeigen/eigen.

The fastcone native kernel uses Eigen for sparse matrix algebra and Cholesky,
not as an optimization solver. Only the `Eigen/` headers and upstream license
files are vendored. Copyright notices remain in each file; see `COPYING.MPL2`
and the other upstream `COPYING.*` files for applicable notices.

The headers are checked in so `fastcone.build` requires no network download.
