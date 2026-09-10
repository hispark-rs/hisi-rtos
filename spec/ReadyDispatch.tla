--------------------------- MODULE ReadyDispatch ---------------------------
EXTENDS Naturals
CONSTANT Legacy
VARIABLES phase, current, ready, pending, armed
vars == <<phase, current, ready, pending, armed>>

\* Bounded counterexample: a timer IRQ arrives after a worker commits a
\* handoff to preemptive main but before the worker can pend its SWI.
Init == /\ phase = 0 /\ current = "worker" /\ ready = FALSE
        /\ pending = FALSE /\ armed = FALSE
Commit == /\ phase = 0 /\ phase' = 1 /\ pending' = TRUE
          /\ UNCHANGED <<current, ready, armed>>
Wake == /\ phase = 1 /\ phase' = 2 /\ ready' = TRUE
        /\ UNCHANGED <<current, pending, armed>>
Consume == /\ phase = 2 /\ phase' = 3 /\ current' = "main"
           /\ pending' = FALSE /\ UNCHANGED <<ready, armed>>
Rearm == /\ phase = 3 /\ phase' = 4 /\ armed' = ~Legacy
         /\ UNCHANGED <<current, ready, pending>>
Dispatch == /\ phase = 4 /\ armed /\ phase' = 5
            /\ current' = "higher" /\ ready' = FALSE /\ armed' = FALSE
            /\ UNCHANGED pending
Next == Commit \/ Wake \/ Consume \/ Rearm \/ Dispatch
ReadyWorkHasDispatchOpportunity ==
    (phase = 4 /\ current = "main" /\ ready /\ ~pending) => armed
Spec == Init /\ [][Next]_vars
\* This proves an armed-progress obligation, not delivery fairness or timing.
\* Hardware interrupt progress and bounded locks remain environment assumptions.
=============================================================================
