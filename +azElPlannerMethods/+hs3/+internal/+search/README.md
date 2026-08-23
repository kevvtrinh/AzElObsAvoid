# HS3 search

This module builds Plan-325's bounded visibility, homology, and timed-search
seed portfolio. Seeds are topology and timing proposals; they are not valid
motions and never establish planner success on their own.

Candidate ordering, finite work limits, and provenance fields are part of the
preserved HS3 behavior. Keep them inside this method rather than sharing the
corridor search implementation.
