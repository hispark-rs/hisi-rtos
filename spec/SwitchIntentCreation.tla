--------------------- MODULE SwitchIntentCreation ---------------------
EXTENDS Integers, TLC

CONSTANT LegacyMode

VARIABLES phase,
          sourceState,
          current,
          targetOwner,
          pending,
          resumeGeneration,
          created,
          committed,
          cancelled,
          completed

vars == << phase, sourceState, current, targetOwner, pending,
          resumeGeneration, created, committed, cancelled, completed >>

Init ==
    /\ phase = "Running"
    /\ sourceState = "Running"
    /\ current = "Source"
    /\ targetOwner = "ReadyQueued"
    /\ pending = FALSE
    /\ resumeGeneration = 0
    /\ created = 0
    /\ committed = 0
    /\ cancelled = 0
    /\ completed = 0

MarkNonRunning ==
    /\ phase = "Running"
    /\ phase' = "MarkedNonRunning"
    /\ sourceState' = "NonRunning"
    /\ UNCHANGED << current, targetOwner, pending, resumeGeneration,
                    created, committed, cancelled, completed >>

LegacyPrecheck ==
    /\ LegacyMode
    /\ phase = "MarkedNonRunning"
    /\ sourceState = "NonRunning"
    /\ phase' = "Prechecked"
    /\ UNCHANGED << sourceState, current, targetOwner, pending,
                    resumeGeneration, created, committed, cancelled, completed >>

IrqSwitchAndResume ==
    /\ phase \in {"MarkedNonRunning", "Prechecked"}
    /\ sourceState = "NonRunning"
    /\ sourceState' = "Running"
    /\ current' = "Source"
    /\ resumeGeneration' = resumeGeneration + 1
    /\ UNCHANGED << phase, targetOwner, pending, created, committed,
                    cancelled, completed >>

PrepareOrObserveResume ==
    /\ ~LegacyMode
    /\ phase = "MarkedNonRunning"
    /\ IF sourceState = "NonRunning"
          THEN /\ phase' = "Committed"
               /\ targetOwner' = "Detached"
               /\ pending' = TRUE
               /\ created' = created + 1
               /\ committed' = committed + 1
               /\ UNCHANGED << sourceState, current, resumeGeneration,
                               cancelled, completed >>
          ELSE /\ phase' = "NoSwitch"
               /\ cancelled' = cancelled + 1
               /\ UNCHANGED << sourceState, current, targetOwner, pending,
                               resumeGeneration, created, committed, completed >>

LegacyPrepareAfterPrecheck ==
    /\ LegacyMode
    /\ phase = "Prechecked"
    /\ phase' = "Committed"
    /\ targetOwner' = "Detached"
    /\ pending' = TRUE
    /\ created' = created + 1
    /\ committed' = committed + 1
    /\ UNCHANGED << sourceState, current, resumeGeneration, cancelled, completed >>

Consume ==
    /\ phase = "Committed"
    /\ pending
    /\ phase' = "Completed"
    /\ current' = "Target"
    /\ targetOwner' = "Running"
    /\ pending' = FALSE
    /\ completed' = completed + 1
    /\ UNCHANGED << sourceState, resumeGeneration, created, committed, cancelled >>

Next ==
    \/ MarkNonRunning
    \/ LegacyPrecheck
    \/ IrqSwitchAndResume
    \/ PrepareOrObserveResume
    \/ LegacyPrepareAfterPrecheck
    \/ Consume

TypeOK ==
    /\ LegacyMode \in BOOLEAN
    /\ phase \in {"Running", "MarkedNonRunning", "Prechecked",
                    "Committed", "NoSwitch", "Completed"}
    /\ sourceState \in {"Running", "NonRunning"}
    /\ current \in {"Source", "Target"}
    /\ targetOwner \in {"ReadyQueued", "Detached", "Running"}
    /\ pending \in BOOLEAN
    /\ resumeGeneration \in 0..1
    /\ created \in 0..1
    /\ committed \in 0..1
    /\ cancelled \in 0..1
    /\ completed \in 0..1

PreparedSourceNotResumed == pending => sourceState = "NonRunning"
NoSwitchPreservesReady == phase = "NoSwitch" =>
    /\ targetOwner = "ReadyQueued"
    /\ ~pending
    /\ created = 0
    /\ committed = 0
PendingOwnsDetachedTarget == pending => targetOwner = "Detached"
PendingIffCommitted == pending <=> phase = "Committed"
UniqueRunning ==
    /\ sourceState = "Running" => current = "Source"
    /\ targetOwner = "Running" => current = "Target"
TerminalConservation ==
    /\ phase = "NoSwitch" => cancelled = 1
    /\ phase = "Completed" => committed = completed

Spec == Init /\ [][Next]_vars

=============================================================================
