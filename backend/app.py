#!/bin/#!/usr/bin/env python3

# Copyright 2021 Raphael Javaux
#
# This file is part of BikeTracker.
#
# BikeTracker is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# CovidTracer is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with CovidTracer. If not, see<https://www.gnu.org/licenses/>.

import os, datetime

from flask import Flask, jsonify, render_template, request
from flask_sqlalchemy import SQLAlchemy

import wtforms

app = Flask(__name__)
app.config.from_object(os.environ['APP_SETTINGS'])

app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

class Probe(db.Model):
    __tablename__ = 'probes'

    id = db.Column(db.Integer, primary_key=True)

    created_at = db.Column(db.DateTime(), nullable=False, default=datetime.datetime.utcnow)

    seq = db.Column(db.Integer, nullable=False, unique=True)

    lat = db.Column(db.Float, nullable=True)
    lng = db.Column(db.Float, nullable=True)

    dist = db.Column(db.Integer, nullable=True)
    alt_gain = db.Column(db.Integer, nullable=True)
    max_speed = db.Column(db.Float, nullable=True)

@app.route('/')
def index():
    """Shows a dashboard with the latest GPS locations of the tracker."""

    probes = Probe.query                    \
        .order_by(Probe.seq.desc())         \
        .all()

    return render_template('index.html', probes=probes)

class ProbeForm(wtforms.Form):
    device = wtforms.StringField('Device ID', [wtforms.validators.InputRequired()])

    seq = wtforms.IntegerField('Sequence number', [wtforms.validators.InputRequired()])

    lat = wtforms.FloatField('Latitude', [wtforms.validators.InputRequired()])
    lng = wtforms.FloatField('Longitude', [wtforms.validators.InputRequired()])

    dist = wtforms.IntegerField('Distance', [wtforms.validators.InputRequired()])
    alt_gain = wtforms.IntegerField('Elevation gain', [wtforms.validators.InputRequired()])
    max_speed = wtforms.IntegerField('Elevation gain', [wtforms.validators.InputRequired()])

    def validate_device(form, field):
        if field.data != os.environ['DEVICE_ID']:
            raise wtforms.ValidationError('Invalid device ID.')

@app.route('/new-probe', methods=['POST'])
def new_probe():
    """Saves a location in the database. Returns a `201 Created` response on success."""

    form = ProbeForm(request.form)

    if form.validate():
        probe = Probe(
            seq=form.seq.data,

            lat=form.lat.data,
            lng=form.lng.data,

            dist=form.dist.data * 16,
            alt_gain=form.alt_gain.data * 2,
            max_speed=form.max_speed.data / 3.0,
        )
        db.session.add(probe)

        db.session.commit()

        return 'Created', 201
    else:
        return 'Bad request', 400

if __name__ == '__main__':
    db.create_all(app=app)
    app.run()
