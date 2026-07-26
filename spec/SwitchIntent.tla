-------------------------- MODULE SwitchIntent --------------------------
EXTENDS Integers, TLC

VARIABLES state,
          pending,
          previousIdentity,
          targetIdentity,
          intentPreviousIdentity,
          intentTargetIdentity,
          previousResume,
          intentPreviousResume,
          targetOwner,
          terminalCount

vars == << state, pending, previousIdentity, targetIdentity,
          intentPreviousIdentity, intentTargetIdentity, previousResume,
          intentPreviousResume, targetOwner, terminalCount >>

Init ==
    /\ state = "Created"
    /\ pending = FALSE
    /\ previousIdentity = 1
    /\ targetIdentity = 1
    /\ intentPreviousIdentity = 1
    /\ intentTargetIdentity = 1
    /\ previousResume = 0
    /\ intentPreviousResume = 0
    /\ targetOwner = "Detached"
    /\ terminalCount = 0

ReusePrevious ==
    /\ state = "Created"
    /\ previousIdentity < 2
    /\ previousIdentity' = previousIdentity + 1
    /\ UNCHANGED << state, pending, targetIdentity, intentPreviousIdentity,
                    intentTargetIdentity, previousResume, intentPreviousResume,
                    targetOwner, terminalCount >>

ReuseTarget ==
    /\ state = "Created"
    /\ targetIdentity < 2
    /\ targetIdentity' = targetIdentity + 1
    /\ targetOwner' = "OtherIdentity"
    /\ UNCHANGED << state, pending, previousIdentity, intentPreviousIdentity,
                    intentTargetIdentity, previousResume, intentPreviousResume,
                    terminalCount >>

ResumePrevious ==
    /\ state = "Created"
    /\ previousResume < 1
    /\ previousResume' = previousResume + 1
    /\ UNCHANGED << state, pending, previousIdentity, targetIdentity,
                    intentPreviousIdentity, intentTargetIdentity,
                    intentPreviousResume, targetOwner, terminalCount >>

Commit ==
    /\ state = "Created"
    /\ previousIdentity = intentPreviousIdentity
    /\ targetIdentity = intentTargetIdentity
    /\ previousResume = intentPreviousResume
    /\ state' = "Committed"
    /\ pending' = TRUE
    /\ UNCHANGED << previousIdentity, targetIdentity, intentPreviousIdentity,
                    intentTargetIdentity, previousResume, intentPreviousResume,
                    targetOwner, terminalCount >>

CancelIdentity ==
    /\ state = "Created"
    /\ \/ previousIdentity # intentPreviousIdentity
       \/ targetIdentity # intentTargetIdentity
    /\ state' = "Cancelled"
    /\ pending' = FALSE
    /\ terminalCount' = terminalCount + 1
    /\ UNCHANGED << previousIdentity, targetIdentity, intentPreviousIdentity,
                    intentTargetIdentity, previousResume, intentPreviousResume,
                    targetOwner >>

CancelStale ==
    /\ state = "Created"
    /\ previousIdentity = intentPreviousIdentity
    /\ targetIdentity = intentTargetIdentity
    /\ previousResume # intentPreviousResume
    /\ state' = "Cancelled"
    /\ pending' = FALSE
    /\ targetOwner' = "ReadyQueued"
    /\ terminalCount' = terminalCount + 1
    /\ UNCHANGED << previousIdentity, targetIdentity, intentPreviousIdentity,
                    intentTargetIdentity, previousResume, intentPreviousResume >>

Consume ==
    /\ state = "Committed"
    /\ pending
    /\ state' = "Consumed"
    /\ pending' = FALSE
    /\ targetOwner' = "Running"
    /\ terminalCount' = terminalCount + 1
    /\ UNCHANGED << previousIdentity, targetIdentity, intentPreviousIdentity,
                    intentTargetIdentity, previousResume, intentPreviousResume >>

Next ==
    \/ ReusePrevious
    \/ ReuseTarget
    \/ ResumePrevious
    \/ Commit
    \/ CancelIdentity
    \/ CancelStale
    \/ Consume

TypeOK ==
    /\ state \in {"Created", "Committed", "Cancelled", "Consumed"}
    /\ pending \in BOOLEAN
    /\ previousIdentity \in 1..2
    /\ targetIdentity \in 1..2
    /\ previousResume \in 0..1
    /\ targetOwner \in {"Detached", "ReadyQueued", "Running", "OtherIdentity"}
    /\ terminalCount \in 0..1

PendingIffCommitted == pending <=> state = "Committed"
TerminalExactlyOnce == state \in {"Cancelled", "Consumed"} => terminalCount = 1
DetachedOwnership ==
    /\ state = "Committed" => targetOwner = "Detached"
    /\ state = "Consumed" => targetOwner = "Running"

Spec == Init /\ [][Next]_vars

=============================================================================
