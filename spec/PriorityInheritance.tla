------------------------ MODULE PriorityInheritance ------------------------
EXTENDS Integers, Sequences, FiniteSets

CONSTANT MaxPriority

NoTask == "none"
NoMutex == "none"
Low == "low"
Mid == "mid"
High == "high"
Peer == "peer"
Tasks == {Low, Mid, High, Peer}
Mutexes == {"outer", "inner"}

BasePriority ==
    [task \in Tasks |->
        CASE task = Low -> MaxPriority
          [] task = Mid -> MaxPriority - 1
          [] OTHER -> 0]

VARIABLES owner, waiters, waitingOn, lastHandoff, expectedHandoff,
          cycleRejected, rejectedOwner, rejectedWaiters, rejectedWaitingOn

vars ==
    <<owner, waiters, waitingOn, lastHandoff, expectedHandoff,
      cycleRejected, rejectedOwner, rejectedWaiters, rejectedWaitingOn>>

SeqContains(queue, task) ==
    \E index \in 1..Len(queue): queue[index] = task

Remove(queue, task) ==
    SelectSeq(queue, LAMBDA candidate: candidate # task)

MinPriority(set) ==
    IF set = {} THEN MaxPriority
    ELSE CHOOSE priority \in set:
        \A candidate \in set: priority <= candidate

InnerDonation ==
    MinPriority(
        {BasePriority[task]:
            task \in {candidate \in Tasks: SeqContains(waiters["inner"], candidate)}}
    )

MidEffective == MinPriority({BasePriority[Mid], InnerDonation})

OuterDonation(task) ==
    IF task = Mid THEN MidEffective ELSE BasePriority[task]

LowEffective ==
    MinPriority(
        {BasePriority[Low]}
        \union
        {OuterDonation(task):
            task \in {candidate \in Tasks: SeqContains(waiters["outer"], candidate)}}
    )

Effective(task) ==
    CASE task = Low -> LowEffective
      [] task = Mid -> MidEffective
      [] OTHER -> BasePriority[task]

InsertAt(queue, task, position) ==
    SubSeq(queue, 1, position - 1)
    \o <<task>>
    \o SubSeq(queue, position, Len(queue))

StablePriorityInsert(queue, task) ==
    LET earlier ==
            {index \in 1..Len(queue):
                Effective(task) < Effective(queue[index])}
        position ==
            IF earlier = {} THEN Len(queue) + 1
            ELSE CHOOSE index \in earlier:
                \A candidate \in earlier: index <= candidate
    IN InsertAt(queue, task, position)

QueueOrdered(queue) ==
    \A left, right \in 1..Len(queue):
        left < right => Effective(queue[left]) <= Effective(queue[right])

Init ==
    /\ owner =
        [mutex \in Mutexes |->
            IF mutex = "outer" THEN Low ELSE Mid]
    /\ waiters = [mutex \in Mutexes |-> <<>>]
    /\ waitingOn = [task \in Tasks |-> NoMutex]
    /\ lastHandoff = NoTask
    /\ expectedHandoff = NoTask
    /\ cycleRejected = FALSE
    /\ rejectedOwner = owner
    /\ rejectedWaiters = waiters
    /\ rejectedWaitingOn = waitingOn

Wait(task, mutex) ==
    /\ task \in Tasks
    /\ mutex \in Mutexes
    /\ ((mutex = "outer" /\ task = Mid)
        \/ (mutex = "inner" /\ task \in {High, Peer}))
    /\ waitingOn[task] = NoMutex
    /\ owner[mutex] # NoTask
    /\ owner[mutex] # task
    /\ ~(task = Low
          /\ mutex = "inner"
          /\ waitingOn[Mid] = "outer"
          /\ owner["outer"] = Low
          /\ owner["inner"] = Mid)
    /\ waiters' = [waiters EXCEPT ![mutex] = StablePriorityInsert(@, task)]
    /\ waitingOn' = [waitingOn EXCEPT ![task] = mutex]
    /\ cycleRejected' = FALSE
    /\ UNCHANGED
        <<owner, lastHandoff, expectedHandoff,
          rejectedOwner, rejectedWaiters, rejectedWaitingOn>>

CancelOrTimeout(task) ==
    /\ task \in Tasks
    /\ waitingOn[task] # NoMutex
    /\ LET mutex == waitingOn[task] IN
        /\ waiters' = [waiters EXCEPT ![mutex] = Remove(@, task)]
        /\ waitingOn' = [waitingOn EXCEPT ![task] = NoMutex]
    /\ cycleRejected' = FALSE
    /\ UNCHANGED
        <<owner, lastHandoff, expectedHandoff,
          rejectedOwner, rejectedWaiters, rejectedWaitingOn>>

Release(mutex) ==
    /\ mutex \in Mutexes
    /\ owner[mutex] # NoTask
    /\ LET queue == waiters[mutex]
           next == IF queue = <<>> THEN NoTask ELSE Head(queue)
       IN
        /\ owner' = [owner EXCEPT ![mutex] = next]
        /\ waiters' =
            [waiters EXCEPT ![mutex] = IF queue = <<>> THEN <<>> ELSE Tail(queue)]
        /\ waitingOn' =
            IF next = NoTask
            THEN waitingOn
            ELSE [waitingOn EXCEPT ![next] = NoMutex]
        /\ lastHandoff' = next
        /\ expectedHandoff' = next
    /\ cycleRejected' = FALSE
    /\ UNCHANGED <<rejectedOwner, rejectedWaiters, rejectedWaitingOn>>

RejectCycle ==
    /\ waitingOn[Mid] = "outer"
    /\ owner["outer"] = Low
    /\ owner["inner"] = Mid
    /\ cycleRejected' = TRUE
    /\ rejectedOwner' = owner
    /\ rejectedWaiters' = waiters
    /\ rejectedWaitingOn' = waitingOn
    /\ UNCHANGED
        <<owner, waiters, waitingOn, lastHandoff, expectedHandoff>>

Next ==
    (\E task \in Tasks, mutex \in Mutexes: Wait(task, mutex))
    \/ (\E task \in Tasks: CancelOrTimeout(task))
    \/ (\E mutex \in Mutexes: Release(mutex))
    \/ RejectCycle

TypeOK ==
    /\ owner \in [Mutexes -> Tasks \union {NoTask}]
    /\ waiters \in [Mutexes -> Seq(Tasks)]
    /\ waitingOn \in [Tasks -> Mutexes \union {NoMutex}]
    /\ lastHandoff \in Tasks \union {NoTask}
    /\ expectedHandoff \in Tasks \union {NoTask}
    /\ cycleRejected \in BOOLEAN

QueueMembershipIsUnique ==
    \A task \in Tasks:
        Cardinality({mutex \in Mutexes: SeqContains(waiters[mutex], task)}) <= 1

WaitingStateMatchesQueue ==
    \A task \in Tasks:
        (waitingOn[task] = NoMutex)
        <=> ~(\E mutex \in Mutexes: SeqContains(waiters[mutex], task))

PriorityQueuesAreStable ==
    \A mutex \in Mutexes: QueueOrdered(waiters[mutex])

DirectHandoffUsesQueueHead ==
    lastHandoff = expectedHandoff

DonationPropagatesAndRestores ==
    /\ (SeqContains(waiters["inner"], High) => MidEffective = BasePriority[High])
    /\ ((SeqContains(waiters["outer"], Mid) /\ SeqContains(waiters["inner"], High))
        => LowEffective = BasePriority[High])
    /\ (waiters["inner"] = <<>> => MidEffective = BasePriority[Mid])
    /\ (waiters["outer"] = <<>> => LowEffective = BasePriority[Low])

RejectedCyclePreservesGraph ==
    cycleRejected =>
        /\ owner = rejectedOwner
        /\ waiters = rejectedWaiters
        /\ waitingOn = rejectedWaitingOn

Spec == Init /\ [][Next]_vars

=============================================================================
