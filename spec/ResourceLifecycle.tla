------------------------- MODULE ResourceLifecycle -------------------------
EXTENDS Naturals

CONSTANT MaxGeneration

VARIABLES live, generation, remembered, grantPending, available,
          staleRejected, duplicateRejected

vars == <<live, generation, remembered, grantPending, available,
          staleRejected, duplicateRejected>>

NextGeneration(g) == IF g = MaxGeneration THEN 1 ELSE g + 1

Init ==
    /\ live = FALSE
    /\ generation = 0
    /\ remembered = 0
    /\ grantPending = FALSE
    /\ available = FALSE
    /\ staleRejected = FALSE
    /\ duplicateRejected = FALSE

Create ==
    /\ ~live
    /\ live' = TRUE
    /\ generation' = NextGeneration(generation)
    /\ grantPending' = FALSE
    /\ available' = FALSE
    /\ staleRejected' = FALSE
    /\ duplicateRejected' = FALSE
    /\ UNCHANGED remembered

Remember ==
    /\ live
    /\ remembered' = generation
    /\ staleRejected' = FALSE
    /\ duplicateRejected' = FALSE
    /\ UNCHANGED <<live, generation, grantPending, available>>

DestroyCurrent ==
    /\ live
    /\ ~grantPending
    /\ ~available
    /\ live' = FALSE
    /\ staleRejected' = FALSE
    /\ duplicateRejected' = FALSE
    /\ UNCHANGED <<generation, remembered, grantPending, available>>

DestroyRemembered ==
    /\ live
    /\ remembered = generation
    /\ ~grantPending
    /\ ~available
    /\ live' = FALSE
    /\ staleRejected' = FALSE
    /\ duplicateRejected' = FALSE
    /\ UNCHANGED <<generation, remembered, grantPending, available>>

RejectStale ==
    /\ live
    /\ remembered # generation
    /\ staleRejected' = TRUE
    /\ duplicateRejected' = FALSE
    /\ UNCHANGED <<live, generation, remembered, grantPending, available>>

RejectDuplicate ==
    /\ ~live
    /\ duplicateRejected' = TRUE
    /\ staleRejected' = FALSE
    /\ UNCHANGED <<live, generation, remembered, grantPending, available>>

IssueDirectGrant ==
    /\ live
    /\ ~grantPending
    /\ ~available
    /\ grantPending' = TRUE
    /\ available' = FALSE
    /\ staleRejected' = FALSE
    /\ duplicateRejected' = FALSE
    /\ UNCHANGED <<live, generation, remembered>>

CancelDirectGrant ==
    /\ grantPending
    /\ grantPending' = FALSE
    /\ available' = TRUE
    /\ staleRejected' = FALSE
    /\ duplicateRejected' = FALSE
    /\ UNCHANGED <<live, generation, remembered>>

AcquireAvailable ==
    /\ available
    /\ available' = FALSE
    /\ staleRejected' = FALSE
    /\ duplicateRejected' = FALSE
    /\ UNCHANGED <<live, generation, remembered, grantPending>>

Next == Create \/ Remember \/ DestroyCurrent \/ DestroyRemembered
        \/ RejectStale \/ RejectDuplicate \/ IssueDirectGrant
        \/ CancelDirectGrant \/ AcquireAvailable

TypeOK ==
    /\ live \in BOOLEAN
    /\ generation \in 0..MaxGeneration
    /\ remembered \in 0..MaxGeneration
    /\ grantPending \in BOOLEAN
    /\ available \in BOOLEAN
    /\ staleRejected \in BOOLEAN
    /\ duplicateRejected \in BOOLEAN

NoGrantWithoutLiveResource == (grantPending \/ available) => live
GrantIsNeverDuplicated == ~(grantPending /\ available)
StaleDestroyPreservesLiveResource == staleRejected => live
DuplicateDestroyPreservesDeadResource == duplicateRejected => ~live

Spec == Init /\ [][Next]_vars

=============================================================================
