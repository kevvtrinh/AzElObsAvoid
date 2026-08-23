# HS3 obstacles

This module converts canonical obstacle histories into the immutable dynamic
geometry used by Plan-325 and answers shape-at-time requests. It preserves the
source branch's original/protected geometry distinction and never applies a
new safety margin.

The cache is local to HS3. It must not be replaced by or passed through the
corridor method's obstacle module.
