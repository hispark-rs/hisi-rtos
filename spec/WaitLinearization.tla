------------------------- MODULE WaitLinearization -------------------------
EXTENDS Naturals, FiniteSets

CONSTANT Tasks

NoTerminal == "none"
SignalTerminal == "signal"
TimeoutTerminal == "timeout"
CancelTerminal == "cancel"
TerminalKinds == {NoTerminal, SignalTerminal, TimeoutTerminal, CancelTerminal}

VARIABLES waiters, ready, grants, terminal, permits, posts, consumes

vars == <<waiters, ready, grants, terminal, permits, posts, consumes>>

Init ==
    /\ waiters = Tasks
    /\ ready = {}
    /\ grants = {}
    /\ terminal = [task \in Tasks |-> NoTerminal]
    /\ permits = 0
    /\ posts = 0
    /\ consumes = 0

SignalWaiter(task) ==
    /\ posts = 0
    /\ task \in waiters
    /\ waiters' = waiters \ {task}
    /\ ready' = ready \cup {task}
    /\ grants' = grants \cup {task}
    /\ posts' = 1
    /\ UNCHANGED <<terminal, permits, consumes>>

SignalPermit ==
    /\ posts = 0
    /\ waiters = {}
    /\ permits' = 1
    /\ posts' = 1
    /\ UNCHANGED <<waiters, ready, grants, terminal, consumes>>

Timeout(task) ==
    /\ task \in waiters
    /\ waiters' = waiters \ {task}
    /\ ready' = ready \cup {task}
    /\ terminal' = [terminal EXCEPT ![task] = TimeoutTerminal]
    /\ UNCHANGED <<grants, permits, posts, consumes>>

CancelQueued(task) ==
    /\ task \in waiters
    /\ waiters' = waiters \ {task}
    /\ ready' = ready \cup {task}
    /\ terminal' = [terminal EXCEPT ![task] = CancelTerminal]
    /\ UNCHANGED <<grants, permits, posts, consumes>>

CancelGrant(task) ==
    /\ task \in grants
    /\ grants' = grants \ {task}
    /\ terminal' = [terminal EXCEPT ![task] = CancelTerminal]
    /\ permits' = permits + 1
    /\ UNCHANGED <<waiters, ready, posts, consumes>>

ConsumeGrant(task) ==
    /\ task \in grants
    /\ grants' = grants \ {task}
    /\ terminal' = [terminal EXCEPT ![task] = SignalTerminal]
    /\ consumes' = consumes + 1
    /\ UNCHANGED <<waiters, ready, permits, posts>>

Next ==
    SignalPermit
    \/ \E task \in Tasks:
        SignalWaiter(task)
        \/ Timeout(task)
        \/ CancelQueued(task)
        \/ CancelGrant(task)
        \/ ConsumeGrant(task)

TypeOK ==
    /\ waiters \subseteq Tasks
    /\ ready \subseteq Tasks
    /\ grants \subseteq Tasks
    /\ terminal \in [Tasks -> TerminalKinds]
    /\ permits \in 0..1
    /\ posts \in 0..1
    /\ consumes \in 0..1

SingleSchedulerOwner ==
    /\ waiters \intersect ready = {}
    /\ waiters \cup ready = Tasks

GrantOnlyReady ==
    grants \subseteq ready

TerminalResultUnique ==
    \A task \in Tasks:
        terminal[task] # NoTerminal =>
            /\ task \in ready
            /\ task \notin waiters
            /\ task \notin grants

WaitingHasNoResolvedResult ==
    \A task \in waiters:
        /\ terminal[task] = NoTerminal
        /\ task \notin grants

ResourceConservation ==
    permits + Cardinality(grants) + consumes = posts

Spec == Init /\ [][Next]_vars

=============================================================================
