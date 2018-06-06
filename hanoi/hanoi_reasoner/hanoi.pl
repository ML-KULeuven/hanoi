:- use_module('dc_planning_bridge.pl').
:- use_module(library(lists)).

% Options
:- set_options(default),
   set_query_propagation(true),
   set_inference(backward(lazy)).
:- set_current2nextcopy(false).

builtin(!).
builtin(break).
builtin(format(_,_)).
builtin(deltaT(_)).
builtin(varQ(_)).
builtin(cov(_,_,_)).
builtin(print_time).
builtin(user:timestep(_)).
builtin(bb_get(_,_)).
builtin(append(_,_,_)).
builtin(delete(_,_,_)).
builtin(sort(_,_)).
builtin(length(_,_)).
builtin(length_of_partial_solution(_,_,_)).
builtin(keysort(_,_)).
builtin(A=B).
builtin(A\==B).
builtin(A==B).
builtin(A\=B).
builtin(A is B).
builtin(A > B).
builtin(A < B).
builtin(A >= B).
builtin(A =< B).
builtin((_ -> _ ; _)).
builtin((_ -> _)). % Needed for if-condition in reward.
builtin((_ ; _)).

satisfiable(A=B).
satisfiable(A\==B).
satisfiable(A\=B).
satisfiable(A > B).
satisfiable(A < B).
satisfiable(A >= B).
satisfiable(A =< B).

builtin(\+A) :- builtin(A).

maxV(D,V):t := V is 1000.

deltaT(1).
varQ(0.0004).
cov(1,[Cov],VarQ) :-
	deltaT(DeltaT),
	Cov is VarQ*DeltaT^2.

tower(X) := member(X, [2, 3, 4]).

disk(X) := member(X, [6, 7, 8]).

% The initial tower
initial_tower(2) := true.
% The helper tower
helper_tower(3) := true.
% The target tower
target_tower(4) := true.

% size/1
% the size of the disk is a gaussian around its id (for testing)
size(X) ~ gaussian(X, 0.01) := disk(X).

% Stop condition: all disks on right tower in order
stop:t := 
    \+action(move(_,_,_,_)),
    target_tower(TT),
    findall_forward(D,on_tower(D):t~=TT,DisksOnRight),
    findall_forward(D,disk(D),AllDisks),
    length(AllDisks, NBDisks),
    length_of_partial_solution(DisksOnRight, AllDisks, L),
    L == NBDisks.

reward:t ~ val(R) := 
    stop:t, R is 1000.
reward:t ~ val(R) := 
    \+stop:t,
    target_tower(TT),
    findall_forward(D,on_tower(D):t~=TT,DisksOnRight),
    action(move(Disk,_,Tower,_)),
    ( % extract to YAP clause
        Tower == TT -> 
            (NextDisksOnRight = [Disk|DisksOnRight])
        ;
            (delete(DisksOnRight, Disk, NextDisksOnRight))
    ),
    findall_forward(D,disk(D),AllDisks),
    length_of_partial_solution(DisksOnRight, AllDisks, L),
    length_of_partial_solution(NextDisksOnRight, AllDisks, NL),
    R is 100 * (NL - L) - 10.

length_of_partial_solution(DisksOnRight, AllDisks, N) :-
    sort(DisksOnRight, SortedDisksOnRight),
    sort(AllDisks, SortedAllDisks),
    (
        append(Tail,SeqSortedDisksOnRight,SortedDisksOnRight),
        append(_,SeqSortedDisksOnRight,SortedAllDisks),
        !
    ),
    length(SeqSortedDisksOnRight, N).

% top_disk/1
% true if X if there is no disk above X on the same tower.
top_disk(X):t := 
    findall_forward(Y,above(Y,X):t,[]).

unify_position(Obj, Pos):t :=
    pos(Obj):t ~= distribution(val(Pos)).

unify_position(Obj, Pos):t :=
    pos(Obj):t ~= Pos.

distance_to(Disk, Tower, Dist):t :=
    unify_position(Disk, (PDX, PDY, PDZ)):t,
    unify_position(Tower,(PTX, PTY, PTZ)):t,
    Dist is sqrt( (PTX-PDX)^2 + (PTY-PDY)^2 + (PTZ-PDZ)^2 ).

on_tower(D):t ~ val(Closest) := 
    disk(D),    
    initial_tower(IT),
    helper_tower(HT1),
    target_tower(TT),
    % left tower
    distance_to(D, IT, DistIT):t,
    % HT1 tower
    distance_to(D, HT1, DistHT1):t,
    % right tower
    distance_to(D, TT, DistTT):t,
    keysort([DistIT-IT,DistHT1-HT1,DistTT-TT], [_-Closest|_]).

% Define admissible actions
adm(action(move(X,pos(PXX,PXY,PXZ),T,pos(PTX,PTY,NZ)))):t :=
    disk(X),tower(T),
    % X has to be the top disk
    top_disk(X):t,
    % X has to be moved to another tower
    on_tower(X):t ~= TX,
    T \== TX,
    % No other disk on T can be smaller than X
    findall_forward(Y,
                    (on_tower(Y):t~=T,smaller(Y,X)),
                    []
    ),
    unify_position(X,(PXX,PXY,PXZ)):t,
    unify_position(T,(PTX,PTY,_)):t,
    findall_forward(Y,on_tower(Y):t~=T,DisksOnT),
    length(DisksOnT, NbDisksOnT),
    NZ is 0.12 + 0.08 * NbDisksOnT.

% above/2
% check whether disk X is on top of disk Y
% deterministically true if X and Y are on the same tower
% and X is smaller than Y.
above(X,Y):t := 
    disk(X), disk(Y),
    on_tower(X):t ~= TX, on_tower(Y):t ~= TY,
    TX == TY,
    smaller(X,Y). % TODO: check Z-coordinates

% smaller/2
% deterministically true if the size of disk X is strictly less than the size of disk Y.
smaller(X,Y) :=
    size(X) ~= SX, 
    size(Y) ~= SY, 
    SX < SY.

% pos/1
pos(ID):0 ~ val(P) :=
	observation(pos(ID)) ~= P.
    
% Position given by observation
pos(ID):t+1 ~ val(P) :=
    observation(pos(ID)) ~= P.

% Measurement model
observation(pos(ID)):t+1 ~ finite([1:_]) := 
    true.

% Position after move action without observation
pos(ID):t+1 ~ indepGaussians([ ([X],[Cov]), ([Y],[Cov]), ([Z],[Cov]) ]) :=
	action(move(ID,_,_,pos(X, Y, Z))),
	varQ(VarQ),
	cov(1,[Cov],VarQ).

% Position without action and no observation
pos(ID):t+1 ~ indepGaussians([ ([X],[Cov]), ([Y],[Cov]), ([Z],[Cov]) ]) :=
	unify_position(ID,(X, Y, Z)):t,
	varQ(VarQ),
	cov(1,[Cov],VarQ).

init :- executedplan_start.

getparam(hanoi) :-
	bb_put(user:spant,0),
	setparam(
        % enable abstraction
        false,
        % ratio of the samples reserved for the first action
        1.0,
        % use correct formula (leave true)
        true,
        % strategy to store V function
        max,
        % ExecAction
        best,
        % most,
        % Domain
        propfalse,
        % relfalse,
        % Discount
        0.95,
        % probability to explore in the beginning (first sample)
        0.0,
        % probability to explore in the end (last sample)
        0.0,
        % number of previous samples to use to estimate Q. Larger is better but slower
        100,
        % max horizon span
        200,
        % lambda init
        0.9,
        % lambda final
        0.9,
        % UCBV
        false,
        % decay
        0.015,
        % action selection: softmax | egreedy
        softmax,
        % egreedy,
        % Pruning
        0,
        % WHeuInit
        -0.1,
        % WHeuFinal
        -0.1),
    !.