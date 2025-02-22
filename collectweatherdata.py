import openmeteo_requests

import requests_cache
import pandas as pd
from retry_requests import retry

# Setup the Open-Meteo API client with cache and retry on error
cache_session = requests_cache.CachedSession('.cache', expire_after = -1)
retry_session = retry(cache_session, retries = 5, backoff_factor = 0.2)
openmeteo = openmeteo_requests.Client(session = retry_session)

# Make sure all required weather variables are listed here
# The order of variables in hourly or daily is important to assign them correctly below
url = "https://archive-api.open-meteo.com/v1/archive"
params = {
	"latitude": 34.0564,
	"longitude": -118.5173,
	"start_date": "2025-01-01",
	"end_date": "2025-01-30",
	"hourly": ["temperature_2m", "relative_humidity_2m", "wind_speed_10m", "wind_direction_10m"],
    "timezone": "America/Los_Angeles",
    "temperature_unit": "fahrenheit"
}
responses = openmeteo.weather_api(url, params=params)

# Process first location. Add a for-loop for multiple locations or weather models
response = responses[0]
print(f"Coordinates {response.Latitude()}°N {response.Longitude()}°E")
print(f"Elevation {response.Elevation()} m asl")
print(f"Timezone {response.Timezone()} {response.TimezoneAbbreviation()}")
print(f"Timezone difference to GMT+0 {response.UtcOffsetSeconds()} s")

# Process hourly data. The order of variables needs to be the same as requested.
hourly = response.Hourly()
hourly_temperature_2m = hourly.Variables(0).ValuesAsNumpy()
hourly_relative_humidity_2m = hourly.Variables(1).ValuesAsNumpy()
hourly_wind_speed_10m = hourly.Variables(2).ValuesAsNumpy()
hourly_rain = hourly.Variables(2).ValuesAsNumpy()
hourly_wind_direction_10m = hourly.Variables(3).ValuesAsNumpy()

hourly_data = {"date": pd.date_range(
	start = pd.to_datetime(hourly.Time(), unit = "s", utc = True),
	end = pd.to_datetime(hourly.TimeEnd(), unit = "s", utc = True),
	freq = pd.Timedelta(seconds = hourly.Interval()),
	inclusive = "left"
)}

hourly_data["temperature_2m"] = hourly_temperature_2m
hourly_data["relative_humidity_2m"] = hourly_relative_humidity_2m
hourly_data["wind_speed_10m"] = hourly_wind_speed_10m
hourly_data["wind_direction_10m"] = hourly_wind_direction_10m
hourly_data["rain"] = hourly_rain


hourly_dataframe = pd.DataFrame(data=hourly_data)

# Convert 'date' to datetime and remove timezone (if needed)
hourly_dataframe['date'] = pd.to_datetime(hourly_dataframe['date'])  # if not already datetime
hourly_dataframe['date'] = hourly_dataframe['date'].dt.tz_convert(None)  # Remove timezone

# Save to Excel
try:
    hourly_dataframe.to_excel("weather_data_openmeteo2.xlsx", index=False)
    print("Data saved to weather_data_openmeteo.xlsx")
except Exception as e:
    print(f"Error saving to Excel: {e}")

# Save to CSV (optional)
try:
    hourly_dataframe.to_csv("weather_data_openmeteo.csv", index=False)
    print("Data saved to weather_data_openmeteo.csv")
except Exception as e:
    print(f"Error saving to CSV: {e}")
