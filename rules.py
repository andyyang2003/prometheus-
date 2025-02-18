# i need to analyze temperature change rate (if big change in temperature, give 10 points to fire)
# rate of change in temperature
# read lets say 20 readings assuming a reading is taken every minute
# subtract R[t] - R[t-1] ) / time for reading -> rate of change from t to t-1 
# if humidity decreases quickly, give certain amount of points to fire
# if high wind speed, give certain points to fire
# ---- EXAMPLE DATA SHEET ----
#Timestamp,tx,t,h,d,lt,ln,gas
#2025-02-17 23:46:24,0.11,25.7,57.5,591.76,0.0,0.0,0
#2025-02-17 23:46:26,0.12,25.7,56.0,611.26,0.0,0.0,0
import pandas as pd

def calculate_temp_change(csv_file):
    try:
        df = pd.read_csv(csv_file)
        df['Timestamp'] = pd.to_datetime(df['Timestamp']) # ensures timestamp is in date time format to apply operations
        df['TimeDiff'] = pd['Timestamp'].diff().dt.total_seconds() # calculates diff between previous rows and converts to seconds
        df['TempChange'] = pd['t'].diff() # calculate diff in temp
        # calculate rate of change between temps
        df['TempROC'] = df['TempChange'] / df['TimeDiff']
    except FileNotFoundError:
        print(f"Error: File not found {csv_file}")
        



