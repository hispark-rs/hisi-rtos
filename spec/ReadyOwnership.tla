------------------------ MODULE ReadyOwnership ------------------------
EXTENDS Integers, TLC

CONSTANT AllowDetachedRequeue

VARIABLES phase,
          sourceState,
          targetState,
          current,
          targetOwner,
          queueCount,
          queueBucket,
          targetPriority,
          pending

vars == << phase, sourceState, targetState, current, targetOwner,
          queueCount, queueBucket, targetPriority, pending >>

Init ==
    /\ phase = "RunningSource"
    /\ sourceState = "Running"
    /\ targetState = "Ready"
    /\ current = "Source"
    /\ targetOwner = "Queued"
    /\ queueCount = 1
    /\ queueBucket = 1
    /\ targetPriority = 1
    /\ pending = FALSE

ChangeQueuedPriority ==
    /\ phase = "RunningSource"
    /\ targetState = "Ready"
    /\ targetOwner = "Queued"
    /\ targetPriority' = 2
    /\ queueBucket' = 2
    /\ UNCHANGED << phase, sourceState, targetState, current, targetOwner,
                    queueCount, pending >>

PrepareSwitch ==
    /\ phase = "RunningSource"
    /\ phase' = "Prepared"
    /\ sourceState' = "Blocked"
    /\ targetOwner' = "Pending"
    /\ queueCount' = 0
    /\ queueBucket' = -1
    /\ pending' = TRUE
    /\ UNCHANGED << targetState, current, targetPriority >>

ChangePendingPriority ==
    /\ phase = "Prepared"
    /\ targetState = "Ready"
    /\ targetOwner = "Pending"
    /\ targetPriority' = 2
    /\ UNCHANGED << phase, sourceState, targetState, current, targetOwner,
                    queueCount, queueBucket, pending >>

LegacyRequeueDetachedTarget ==
    /\ AllowDetachedRequeue
    /\ phase = "Prepared"
    /\ targetState = "Ready"
    /\ targetOwner = "Pending"
    /\ queueCount' = 1
    /\ queueBucket' = targetPriority
    /\ UNCHANGED << phase, sourceState, targetState, current, targetOwner,
                    targetPriority, pending >>

ConsumeSwitch ==
    /\ phase = "Prepared"
    /\ pending
    /\ queueCount = 0
    /\ phase' = "RunningTarget"
    /\ targetState' = "Running"
    /\ current' = "Target"
    /\ targetOwner' = "Running"
    /\ pending' = FALSE
    /\ UNCHANGED << sourceState, queueCount, queueBucket, targetPriority >>

Next ==
    \/ ChangeQueuedPriority
    \/ PrepareSwitch
    \/ ChangePendingPriority
    \/ LegacyRequeueDetachedTarget
    \/ ConsumeSwitch

TypeOK ==
    /\ AllowDetachedRequeue \in BOOLEAN
    /\ phase \in {"RunningSource", "Prepared", "RunningTarget"}
    /\ sourceState \in {"Running", "Blocked"}
    /\ targetState \in {"Ready", "Running"}
    /\ current \in {"Source", "Target"}
    /\ targetOwner \in {"Queued", "Pending", "Running"}
    /\ queueCount \in 0..2
    /\ queueBucket \in {-1, 1, 2}
    /\ targetPriority \in {1, 2}
    /\ pending \in BOOLEAN

ReadyHasExactlyOneOwner ==
    targetState = "Ready" => queueCount + (IF pending THEN 1 ELSE 0) = 1

QueueMembershipUnique == queueCount <= 1

QueuePriorityMatches == queueCount = 1 => queueBucket = targetPriority

CurrentRunningConsistent ==
    /\ phase = "RunningSource" => /\ sourceState = "Running"
                                    /\ current = "Source"
    /\ phase = "Prepared" => /\ sourceState = "Blocked"
                               /\ targetState = "Ready"
                               /\ current = "Source"
                               /\ pending
    /\ phase = "RunningTarget" => /\ targetState = "Running"
                                    /\ current = "Target"
                                    /\ ~pending

Spec == Init /\ [][Next]_vars

=======================================================================
