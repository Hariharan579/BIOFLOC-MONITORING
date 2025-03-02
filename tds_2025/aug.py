import numpy as np
import pandas as pd

# Load your original dataset from a CSV file
file_path = "dataset.csv"
original_data = pd.read_csv(file_path, header=None)

# Number of samples you want to generate
desired_samples = 15000

# Initialize an empty array to store augmented data
augmented_data = []

# Perform data augmentation until the desired number of samples is reached
while len(augmented_data) < desired_samples:
    # Choose a random index from the original dataset
    idx = np.random.randint(0, len(original_data))

    # Extract features and label from the chosen index
    selected_row = original_data.iloc[idx]
    features = selected_row.iloc[:-1]  # Exclude the last column (label)
    label = selected_row.iloc[-1]

    # Check if all values in the features column can be converted to float
    try:
        features = features.astype(float)
    except ValueError:
        # Skip the row if there are non-numeric values
        continue

    # Apply random augmentation techniques to features
    augmented_features = features + np.random.normal(0, 0.1, size=len(features))

    # Convert the label to a string and combine with the augmented features
    augmented_sample = np.concatenate((augmented_features, [str(label)]))

    # Append the augmented sample to the dataset
    augmented_data.append(augmented_sample)

# Convert the list to a numpy array with object dtype (to handle strings)
augmented_data = np.array(augmented_data, dtype=object)

# Print the shape of the augmented dataset
print("Shape of augmented dataset:", augmented_data.shape)

# Save the augmented dataset to a file if needed
np.savetxt("data.csv", augmented_data, delimiter=",", fmt="%s")
