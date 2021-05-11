# -*- coding: utf-8 -*-
from flask import flash
from datetime import datetime
import urllib
import json
import random

class weather(object):

    thingsSpeakChannel = ["1274150", ""]
    thingsSpeakKey = ["NJFP7YQNYOT9C9H4", ""]
    stationName = ["York St John Campus", ""]
    
    #Translating degrees to cardinal directions
    def getWindDir(windDegree):

        windDegree = int(windDegree)
        if windDegree > 330 or windDegree < 30:
            return "N"
        if windDegree > 30 and windDegree < 60:
            return "NE"
        if windDegree > 60 and windDegree < 120:
            return "E"
        if windDegree > 120 and windDegree < 150:
            return "SE"
        if windDegree > 150 and windDegree < 210:
            return "S"
        if windDegree > 210 and windDegree < 240:
            return "SW"
        if windDegree > 240 and windDegree < 300:
            return "W"
        else: return "NW"

    #Deciding on the current weather condition
    def getWeatherCondition(weatherData):
        
        #weatherData: 0 = Temp, 1 = Humidity, 2 = Pressure, 3 = Light, 4 = CarbonM, 5 = Wind Speed, 6 = Wind Direction, 7 = Rain
        #Conditions: Sunny, Cloudy, Rainy, Windy
        if int(weatherData[0]["rain"]) > 0:
            return "Rainy"
        elif int(weatherData[0]["windS"]) > 20:
            return "Windy"
        elif int(weatherData[0]["light"]) > 0.1:
            return "Sunny"
        else:
            return "Cloudy"


    #Requesting data from the thingspeak application for the respectable station
    def getValues(station):
        
        data = []
        #request and process JSON into a list
        URL = "https://api.thingspeak.com/channels/"+ weather.thingsSpeakChannel[station] +"/feeds.json?api_key="+ weather.thingsSpeakKey[station] +"&results=1"
        response = urllib.request.urlopen(URL)
        values = json.loads(response.read())
        windDirection = weather.getWindDir(values['feeds'][0]['field7'])
        #Sorted through data from the request is placed into a list
        data = [

            {'temperature': values['feeds'][0]['field1'],
            'humidity': values['feeds'][0]['field2'],
            'pressure': values['feeds'][0]['field3'],
            'light': float(values['feeds'][0]['field4'])/100,
            'carbon': float(values['feeds'][0]['field5'])/100,
            'windS': values['feeds'][0]['field6'],
            'windD': windDirection,
            'rain': values['feeds'][0]['field8']}

        ]
        return (data)

    #Gathering meta data
    def getMeta(weatherData, station):

        now = datetime.now()

        meta = [

            {'stationName': weather.stationName[station],
            'weatherCondition': weather.getWeatherCondition(weatherData),
            'day': now.strftime("%A"),
            'timedate': now.strftime("%d/%m/%Y %H:%M:%S")}

        ]

        return (meta)
        

