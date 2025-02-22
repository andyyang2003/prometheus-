import math
FFMC_yesterday = 0
rain = 0
humidity = 0
temperature = 0
wind_speed = 0
FFMContent_yesterday = 147.2 * (101 - FFMC_yesterday) / (59.5 + FFMC_yesterday)
if FFMContent_yesterday <= 150:
    FFMContentrain_today = FFMContent_yesterday + 42.5 * (rain - 0.5) * math.e**(-100/(251-FFMContent_yesterday)) * (1-math.e**(-6.93/rain - 0.5))
elif FFMContent_yesterday > 150:
    FFMContentrain_today = FFMContent_yesterday + 42.5 * (rain - 0.5) * math.e**(-100/(251-FFMContent_yesterday)) * (1-math.e**(-6.93/rain - 0.5)) + 0.0015 * (FFMContent_yesterday - 150)**2 * (rain-0.5)**2
if FFMContentrain_today > 250:
    FFMContentrain_today = 250

FFMContent_Dry = 0.942 * humidity**0.679 + 11*(math.e**((humidity-100)/10)) + 0.18*(21.1-temperature)*(1-math.e**(-0.115*humidity))

if FFMContent_Dry < FFMContent_yesterday:
    drying_rate_interim = 0.424 * (1-(humidity/100)**1.7) + 0.0694 + wind_speed**0.5 *(1-(humidity/100)**8)
    drying_rate = drying_rate_interim * 0.581 * math.e**(0.0365-temperature)

FFMContent_today = FFMContent_Dry + (FFMContent_yesterday - FFMContent_Dry) * 10 **(-drying_rate)  