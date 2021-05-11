# -*- coding: utf-8 -*-
from project import app
from flask import render_template, request, jsonify
from flask_wtf import FlaskForm
from wtforms import StringField
from wtforms.validators import DataRequired
from project.models.weather_model import weather


class CreateForm(FlaskForm):
    text = StringField('name', validators=[DataRequired()])

#request for index/landing page
@app.route('/')
def start():
    return render_template('index.html')

#client request to update data
@app.route('/data_update')
def update():
    try:
        stationNumber = request.args.get('stationNumber', '0', type=int)
        weatherData = weather.getValues(stationNumber)
        metaData = weather.getMeta(weatherData, stationNumber)
        return jsonify(results = weatherData + metaData)
    except:
        pass

