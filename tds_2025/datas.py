import numpy as np
import pandas as pd

# Set seed for reproducibility
np.random.seed(42)

# Number of data points
num_samples = 1000

# Generate random data for each feature
temperature = np.random.uniform(28, 32, num_samples)  # Temperature range between 28 and 32
gsr = np.random.uniform(2, 22, num_samples)           # GSR range between 2 and 22
ppg = np.random.uniform(60, 120, num_samples)         # PPG range typical for heart rate (in bpm)
ph = np.random.uniform(6.8, 7.4, num_samples)         # pH level close to neutral body pH

# Function to calculate hydration level based on other features
def calculate_hydration(temp, gsr, ppg, ph):
    # Simple formula to estimate hydration level based on features
    hydration = 100 - (0.5 * temp) - (0.2 * gsr) + (0.1 * ppg) - (5 * (7.4 - ph))
    # Ensure hydration level is within 0 to 100%
    return max(0, min(100, hydration))

# Apply the function to calculate the target hydration level
hydration_level = [calculate_hydration(temp, gsr_val, ppg_val, ph_val) 
                   for temp, gsr_val, ppg_val, ph_val in zip(temperature, gsr, ppg, ph)]

# Create a DataFrame
data = pd.DataFrame({
    'Temperature (°C)': temperature,
    'GSR (µS)': gsr,
    'PPG (bpm)': ppg,
    'pH': ph,
    'Hydration Level (%)': hydration_level
})

# Display the first few rows of the dataset
print(data.head())

# Save to a CSV file
data.to_csv('dataset.csv', index=False)
