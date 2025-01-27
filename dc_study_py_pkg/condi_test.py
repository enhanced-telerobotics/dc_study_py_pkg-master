import json
import random
from itertools import product

# Define the values for delay, distance, and direction
delay_values = [100, 400, 700]  # For real trials
dist_values = [0.005, 0.01, 0.015]
dir_values = ['up', 'right', 'diag']
zero_delay = 0  # For catch trials

# Generate all possible 3x3x3 combinations (delay, distance, direction)
conditions_3x3x3 = list(product(delay_values, dist_values, dir_values))

# Ensure each condition appears exactly 5 times for training task
real_trials_pool = [
    {"delay": condition[0], "distance": condition[1], "direction": condition[2]}
    for condition in conditions_3x3x3
    for _ in range(5)
]

# Shuffle the pool of real trials
random.shuffle(real_trials_pool)

# Generate practice conditions
practice_conditions = [
    {
        "trial_num": i + 1,
        "delay": random.choice(delay_values),
        "distance": random.choice(dist_values),
        "direction": random.choice(dir_values)
    }
    for i in range(5)
]

# Generate baseline conditions
baseline_comb = list(product(dist_values, dir_values))
baseline_conditions = [
    {
        "trial_num": i + 1 + 5,
        "delay": zero_delay,
        "distance": baseline_comb[i % len(baseline_comb)][0],
        "direction": baseline_comb[i % len(baseline_comb)][1]
    }
    for i in range(27)
]

# Initialize training task conditions
training_task_conditions = []

# Total number of blocks for training task
num_blocks_training_task = len(real_trials_pool) // 3  # 3 real trials per block

# Assign trials to training task blocks
trial_number = 33  # Start trial numbering after practice and baseline trials
for block_num in range(1, num_blocks_training_task + 1):
    # Take 3 real trials from the pool
    real_trials = real_trials_pool[:3]
    real_trials_pool = real_trials_pool[3:]  # Remove used trials

    # Create 2 catch trials
    catch_trials = [
        {
            "block_num": block_num,
            "delay": zero_delay,
            "distance": random.choice(dist_values),
            "direction": random.choice(dir_values)
        }
        for _ in range(2)
    ]

    # Combine real and catch trials, then shuffle
    block_trials = real_trials + catch_trials
    random.shuffle(block_trials)

    # Assign trial numbers and block numbers
    for trial in block_trials:
        trial['block_num'] = block_num
        trial['trial_num'] = trial_number
        trial_number += 1

    # Add the block's trials to the training task conditions
    training_task_conditions.extend(block_trials)

# Ensure each condition appears exactly 3 times for evaluation task
eval_trials_pool = [
    {"delay": condition[0], "distance": condition[1], "direction": condition[2]}
    for condition in conditions_3x3x3
    for _ in range(3)
]

# Shuffle the pool of evaluation trials
random.shuffle(eval_trials_pool)

# Initialize evaluation task conditions
evaluation_task_conditions = []

# Total number of blocks for evaluation task
num_blocks_evaluation_task = len(eval_trials_pool) // 3  # 3 real trials per block

# Assign trials to evaluation task blocks
for block_num in range(1, num_blocks_evaluation_task + 2):  # +2 to account for any remaining trials
    # Take 3 real trials from the pool (or fewer if fewer than 3 remain)
    real_trials = eval_trials_pool[:3]
    eval_trials_pool = eval_trials_pool[3:]  # Remove used trials

    # Create 1 catch trial
    catch_trials = [
        {
            "block_num": block_num,
            "delay": zero_delay,
            "distance": random.choice(dist_values),
            "direction": random.choice(dir_values)
        }
    ]

    # Combine real and catch trials, then shuffle
    block_trials = real_trials + catch_trials
    random.shuffle(block_trials)

    # Assign trial numbers and block numbers
    for trial in block_trials:
        trial['block_num'] = block_num
        trial['trial_num'] = trial_number
        trial_number += 1

    # Add the block's trials to the evaluation task conditions
    evaluation_task_conditions.extend(block_trials)

# Construct the complete conditions dictionary
conditions = {
    "practice": practice_conditions,
    "baseline": baseline_conditions,
    "training_task": training_task_conditions,
    "evaluation_task": evaluation_task_conditions
}

# Save the conditions to a JSON file
file_path = "trial_con111ditions.json"
with open(file_path, 'w') as json_file:
    json.dump(conditions, json_file, indent=4)

print(f"Trial conditions file generated and saved to {file_path}")