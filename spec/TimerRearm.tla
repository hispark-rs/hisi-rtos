--------------------------- MODULE TimerRearm ---------------------------
EXTENDS Integers, TLC

CONSTANT Actors

VARIABLES currentGeneration,
          desiredDeadline,
          programmedDeadline,
          ticket,
          computedDeadline,
          phase,
          stable,
          validatedTicket

vars == << currentGeneration, desiredDeadline, programmedDeadline,
          ticket, computedDeadline, phase, stable, validatedTicket >>

Init ==
    /\ currentGeneration = 0
    /\ desiredDeadline = 3
    /\ programmedDeadline = 0
    /\ ticket = [actor \in Actors |-> 0]
    /\ computedDeadline = [actor \in Actors |-> 0]
    /\ phase = [actor \in Actors |-> "Idle"]
    /\ stable = FALSE
    /\ validatedTicket = 0

Claim(actor, deadline) ==
    /\ phase[actor] \in {"Idle", "Retry"}
    /\ currentGeneration < 4
    /\ currentGeneration' = currentGeneration + 1
    /\ desiredDeadline' = deadline
    /\ ticket' = [ticket EXCEPT ![actor] = currentGeneration']
    /\ computedDeadline' = [computedDeadline EXCEPT ![actor] = deadline]
    /\ phase' = [phase EXCEPT ![actor] = "Claimed"]
    /\ stable' = FALSE
    /\ UNCHANGED << programmedDeadline, validatedTicket >>

Program(actor) ==
    /\ phase[actor] = "Claimed"
    /\ programmedDeadline' = computedDeadline[actor]
    /\ phase' = [phase EXCEPT ![actor] = "Programmed"]
    /\ stable' = FALSE
    /\ UNCHANGED << currentGeneration, desiredDeadline, ticket,
                    computedDeadline, validatedTicket >>

ValidateCurrent(actor) ==
    /\ phase[actor] = "Programmed"
    /\ ticket[actor] = currentGeneration
    /\ programmedDeadline = desiredDeadline
    /\ phase' = [phase EXCEPT ![actor] = "Done"]
    /\ stable' = TRUE
    /\ validatedTicket' = ticket[actor]
    /\ UNCHANGED << currentGeneration, desiredDeadline, programmedDeadline,
                    ticket, computedDeadline >>

RejectStale(actor) ==
    /\ phase[actor] = "Programmed"
    /\ ticket[actor] # currentGeneration
    /\ phase' = [phase EXCEPT ![actor] = "Retry"]
    /\ UNCHANGED << currentGeneration, desiredDeadline, programmedDeadline,
                    ticket, computedDeadline, stable, validatedTicket >>

Next ==
    \/ \E actor \in Actors, deadline \in 1..3 : Claim(actor, deadline)
    \/ \E actor \in Actors : Program(actor)
    \/ \E actor \in Actors : ValidateCurrent(actor)
    \/ \E actor \in Actors : RejectStale(actor)

TypeOK ==
    /\ currentGeneration \in 0..4
    /\ desiredDeadline \in 1..3
    /\ programmedDeadline \in 0..3
    /\ ticket \in [Actors -> 0..4]
    /\ computedDeadline \in [Actors -> 0..3]
    /\ phase \in [Actors -> {"Idle", "Claimed", "Programmed", "Retry", "Done"}]
    /\ stable \in BOOLEAN
    /\ validatedTicket \in Nat

StableProgrammingIsCurrent ==
    stable => /\ validatedTicket = currentGeneration
              /\ programmedDeadline = desiredDeadline

Spec == Init /\ [][Next]_vars

=============================================================================
