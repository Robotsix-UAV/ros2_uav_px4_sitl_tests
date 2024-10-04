import pandas as pd
import os

# Define the folder containing CSV files
folder_path = '../log/2024-10-01-08-10-42'

# Function to remove outliers based on the IQR method for the timestamp column
def remove_timestamp_outliers(df, column):
    Q1 = df[column].quantile(0.25)
    Q3 = df[column].quantile(0.75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    return df[(df[column] >= lower_bound) & (df[column] <= upper_bound)]

# Loop through all CSV files in the folder
for filename in os.listdir(folder_path):
    if filename.endswith('.csv'):
        file_path = os.path.join(folder_path, filename)
        
        # Load the data
        data = pd.read_csv(file_path)
        
        # Convert timestamp to a numeric format if not already, and sort the data
        data['timestamp'] = pd.to_numeric(data['timestamp'], errors='coerce')
        data = data.sort_values(by='timestamp')
        
        # Remove outliers from the timestamp column
        cleaned_data = remove_timestamp_outliers(data, 'timestamp')
        
        # Save the cleaned data to a new CSV file
        cleaned_file_path = os.path.join(folder_path, f'cleaned_{filename}')
        cleaned_data.to_csv(cleaned_file_path, index=False)
        
        print(f'Cleaned data saved to {cleaned_file_path}')
