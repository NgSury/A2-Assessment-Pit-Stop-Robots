#Constants in pitstop

dt =0.05 # time step for simulation
steps = 50
sequence_transits = 12 #how many joints move in the scene
random_obstacle_count = 3 #how many random obstacles to drop in the scene
car_footprint = ((-2.6,-1.2),(2.9,1.2))
kuka_base_posXY = (4, -1)

#make the bases for the tyre stands
stand_height = 0.25
stand_radius = 0.3
stand_offset = {
    "old": (-0.65, -0.1),
    "new": (-.1,-.65)
}
tyre_stand_grey = [0.55, 0.55, 0.6, 1.0]
tool_grey = [0.3, 0.3, 0.3, 1.0]
dark_grey = [0.2, 0.2, 0.2, 1.0]
floor_grey = [0.35, 0.35, 0.38, 1.0]