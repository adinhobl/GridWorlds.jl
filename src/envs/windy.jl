# Code From JuliaReinforcementLearningAnIntroduction
# https://github.com/JuliaReinforcementLearning/ReinforcementLearningAnIntroduction.jl/blob/master/src/environments/WindyGridWorld.jl

export WindyGridWorld

mutable struct WindyGridWorld <: AbstractGridWorld
    world::GridWorldBase{Tuple{Empty,Wall,Goal}}
    agent_pos::CartesianIndex{2}
    agent::Agent
    goal_reward::Float64
    reward::Float64
    wind::Array{CartesianIndex{2},1}
end

function WindyGridWorld(
    ;width=12, height=9, 
    agent_start_pos=CartesianIndex(5,2), 
    goal_pos=CartesianIndex(5,width-3),
    wind=[CartesianIndex(w, 0) for w in [0, 0, 0, 0, -1, -1, -1, -2, -2, -1, 0, 0]]
    )

    objects = (EMPTY, WALL, GOAL)
    world = GridWorldBase(objects, height, width)
    world[EMPTY,:,:] .= true
    world[WALL,[1,end],:] .= true
    world[WALL,:,[1,end]] .= true
    world[GOAL,goal_pos] = true
    goal_reward = 1.0
    reward = 0.0

    return WindyGridWorld(world, agent_start_pos, Agent(dir=RIGHT), goal_reward, reward, wind)
end

##### Working

function (w::WindyGridWorld)(::MoveForward)
    dir = get_dir(w.agent) 
    att = dir(w.agent_pos) # where agent attempts to go
    ws = w.wind[att[2]] # how the wind in that area will push it

    dest = CartesianIndex(min(max(att[1]+ws[1], 2), size(w.world)[2]-1), 
                          min(max(att[2]+ws[2], 2), size(w.world)[3]-1)
                         ) # bounds checking

    # println(att[1]+ws[1], ", ", att[2]+ws[2], " -> ",
    #         min(max(att[1]+ws[1], 2), size(w.world)[2]-1), ", ",
    #         min(max(att[2]+ws[2], 2), size(w.world)[3]-1))

    w.reward = 0.0  
    if !w.world[WALL, dest]
        w.agent_pos = dest
        if w.world[GOAL, w.agent_pos]
            w.reward = w.goal_reward
        end
    end
    w
end

function (w::WindyGridWorld)(action::Union{TurnRight, TurnLeft})
    w.reward = 0.0
    agent = get_agent(w)
    set_dir!(agent, action(get_dir(agent)))
    w
end

RLBase.get_terminal(w::WindyGridWorld) = w.world[GOAL, w.agent_pos]

RLBase.get_reward(w::WindyGridWorld) = w.reward

function RLBase.reset!(w::WindyGridWorld)
    w.reward = 0.0
    w.agent_pos = CartesianIndex(Int(round(size(w.world)[2]/2, RoundUp)), 2)
    w.agent.dir = RIGHT
    return w
end