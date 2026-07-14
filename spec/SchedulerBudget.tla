-------------------------- MODULE SchedulerBudget --------------------------
EXTENDS Naturals

CONSTANTS Capacity, Period, MaxTime

VARIABLES now, bState, cState, remaining, replenishesAt, lockDepth, pending

vars == <<now, bState, cState, remaining, replenishesAt, lockDepth, pending>>

TaskState == {"Ready", "Running", "Throttled"}

Init ==
    /\ now = 0
    /\ bState = "Running"
    /\ cState = "Ready"
    /\ remaining = Capacity
    /\ replenishesAt = Period
    /\ lockDepth = 0
    /\ pending = FALSE

Tick ==
    /\ now < MaxTime
    /\ now' = now + 1
    /\ IF bState = "Running" /\ remaining > 0
          THEN /\ remaining' = remaining - 1
               /\ IF remaining = 1
                     THEN IF lockDepth = 0
                             THEN /\ bState' = "Throttled"
                                  /\ cState' = "Running"
                                  /\ pending' = FALSE
                             ELSE /\ bState' = bState
                                  /\ cState' = cState
                                  /\ pending' = TRUE
                     ELSE /\ UNCHANGED <<bState, cState, pending>>
          ELSE /\ UNCHANGED <<bState, cState, remaining, pending>>
    /\ UNCHANGED <<replenishesAt, lockDepth>>

Lock ==
    /\ bState = "Running"
    /\ lockDepth < 2
    /\ lockDepth' = lockDepth + 1
    /\ UNCHANGED <<now, bState, cState, remaining, replenishesAt, pending>>

Unlock ==
    /\ lockDepth > 0
    /\ lockDepth' = lockDepth - 1
    /\ IF lockDepth = 1 /\ pending
          THEN /\ bState' = "Throttled"
               /\ cState' = "Running"
               /\ remaining' = 0
               /\ replenishesAt' = IF replenishesAt > now
                                      THEN replenishesAt
                                      ELSE now + Period
               /\ pending' = FALSE
          ELSE /\ UNCHANGED <<bState, cState, remaining, replenishesAt, pending>>
    /\ UNCHANGED now

Replenish ==
    /\ bState = "Throttled"
    /\ now >= replenishesAt
    /\ bState' = "Ready"
    /\ remaining' = Capacity
    /\ replenishesAt' = replenishesAt + Period
    /\ UNCHANGED <<now, cState, lockDepth, pending>>

DispatchBudgeted ==
    /\ bState = "Ready"
    /\ bState' = "Running"
    /\ cState' = "Ready"
    /\ UNCHANGED <<now, remaining, replenishesAt, lockDepth, pending>>

Next == Tick \/ Lock \/ Unlock \/ Replenish \/ DispatchBudgeted

TypeOK ==
    /\ now \in Nat
    /\ bState \in TaskState
    /\ cState \in {"Ready", "Running"}
    /\ remaining \in 0..Capacity
    /\ replenishesAt \in Nat
    /\ lockDepth \in Nat
    /\ pending \in BOOLEAN

OneRunning == (bState = "Running") # (cState = "Running")
BudgetBound == remaining <= Capacity
ExhaustedNotEligible == (remaining = 0 /\ ~pending) => bState = "Throttled"
LockCannotEraseExhaustion == pending => (bState = "Running" /\ lockDepth > 0)

Spec == Init /\ [][Next]_vars

=============================================================================
