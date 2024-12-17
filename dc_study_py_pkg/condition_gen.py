import json
import random
from itertools import product
# Define the values for delay, distance, and direction
delay_values = [100, 400, 700]  # For real trials
zero_delay = 0  # For catch trials
dist_values = [0.005, 0.01, 0.015]
dir_values = ['up', 'right', 'diag']

# Number of blocks for the real task
num_blocks_training_task = 45
trials_per_block = 5  # Each block has 3 real trials and 2 catch trials
baseline_comb = list(product(dist_values, dir_values))
block_comb = list(product(delay_values, dist_values, dir_values))

# Generate baseline conditions
practice_conditions = [
    {
        "trial_num": i + 1,
        "delay": random.choice(delay_values),
        "distance": random.choice(dist_values),
        "direction": random.choice(dir_values)
    }
    for i in range(5)
]

baseline_conditions = [
    {
        "trial_num": i + 1 + 5,
        "delay": zero_delay,
        "distance": baseline_comb[i % len(baseline_comb)][0],
        "direction": baseline_comb[i % len(baseline_comb)][1]
    }
    for i in range(27)
]

# Generate real task conditions with 45 blocks
training_task_conditions = []

trial_number = 33  # Initialize a counter for trial numbers

for block in range(num_blocks_training_task):
    random.shuffle(block_comb)

    # Create 3 real trials with non-zero delay
    real_trials = [
        {
            "block_num": block + 1,
            "delay": block_comb[trial % len(block_comb)][0],
            "distance": block_comb[trial % len(block_comb)][1],
            "direction": block_comb[trial % len(block_comb)][2]
        }
        for trial in range(3)
    ]

    # Create 2 catch trials with zero delay
    catch_trials = [
        {
            "block_num": block + 1,
            "delay": zero_delay,
            "distance": random.choice(dist_values),
            "direction": random.choice(dir_values)
        }
        for trial in range(2)
    ]

    # Combine and shuffle the trials within the block
    block_trials = real_trials + catch_trials
    random.shuffle(block_trials)

    # Assign sequential trial numbers to the shuffled block_trials
    for trial in block_trials:
        trial['trial_num'] = trial_number
        trial_number += 1

    # Extend the main list with the block trials
    training_task_conditions.extend(block_trials)


# Construct the complete conditions dictionary
conditions = {
    "practice": practice_conditions,
    "baseline": baseline_conditions,
    "training_task": training_task_conditions
}

# Save the conditions to a JSON file
file_path = "trial_conditions.json"
with open(file_path, 'w') as json_file:
    json.dump(conditions, json_file, indent=4)

print(f"Randomized baseline conditions file generated and saved to {file_path}")
