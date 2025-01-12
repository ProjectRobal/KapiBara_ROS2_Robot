from gymnasium.envs.registration import register

register(
     id="gym/Maze-v0",
     entry_point="gym.environments.labirynth:Labirynth"
     # kwargs={'maze': 'basic'}
)

register(
     id="gym/Collect-v0",
     entry_point="gym.environments.collect:Collect"
)

register(
     id="gym/Parking-v0",
     entry_point="gym.environments.parking:Parking"
)

register(
     id="gym/Catch-v0",
     entry_point="gym.environments.catch:Catch",
     kwargs={'max_distance':4,'mouse_speed':0.25}
)

register(
     id="gym/Follow-v0",
     entry_point="gym.environments.follow:Follow",
     kwargs={'min_distance':1.5,'max_distance':4}
)