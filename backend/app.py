#!/usr/bin/env python3

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

from typing import Optional

import gpxpy, gpxpy.gpx, pytz, wtforms

from flask import Flask, jsonify, render_template, request
from flask_sqlalchemy import SQLAlchemy

# Will finish an actvity if there was no movement for more than 20 minutes.
INACTIVITY_DELAY = datetime.timedelta(minutes=20)

app = Flask(__name__)
app.config.from_object(os.environ['APP_SETTINGS'])

app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

timezone = pytz.timezone(os.environ.get('TIMEZONE', 'UTC'))

class Activity(db.Model):
    __tablename__ = 'activities'

    id = db.Column(db.Integer, primary_key=True)

    probes = db.relationship('Probe', backref='activity', order_by='Probe.id')

    @property
    def started_at_local(self) -> datetime.datetime:
        return self.probes[0].received_at_local

    @property
    def ended_at_local(self) -> datetime.datetime:
        return self.probes[-1].received_at_local

    @property
    def duration(self) -> datetime.timedelta:
        return self.ended_at_local - self.started_at_local

    @property
    def total_distance(self) -> int:
        """Total distance in meters."""
        return sum(p.dist for p in self.probes)

    @property
    def total_alt_gain(self) -> int:
        """Total altitude gain in meters."""
        return sum(p.alt_gain for p in self.probes)

    @property
    def max_speed(self) -> int:
        """Maximum speed in meters per second."""
        return max(p.max_speed for p in self.probes)


class Probe(db.Model):
    __tablename__ = 'probes'

    id = db.Column(db.Integer, primary_key=True)

    received_at = db.Column(db.DateTime(), nullable=False, default=datetime.datetime.utcnow)

    seq = db.Column(db.Integer, nullable=False, unique=True)

    lat = db.Column(db.Float, nullable=True)
    lng = db.Column(db.Float, nullable=True)
    alt = db.Column(db.Integer, nullable=True)      # m

    dist = db.Column(db.Integer, nullable=True)     # m
    alt_gain = db.Column(db.Integer, nullable=True) # m
    max_speed = db.Column(db.Float, nullable=True)  # m/s

    activity_id = db.Column(db.Integer, db.ForeignKey('activities.id'), nullable=True)

    @property
    def is_idle(self):
        return self.dist == 0;

    @property
    def received_at_local(self) -> datetime.datetime:
        return self.received_at.replace(tzinfo=pytz.UTC).astimezone(tz=timezone)

@app.route('/')
def index():
    """Shows a dashboard with the latest GPS locations of the tracker."""

    probes = Probe.query                    \
        .order_by(Probe.seq.desc())         \
        .all()

    return render_template('index.html', timezone=timezone, probes=probes)

class ProbeForm(wtforms.Form):
    device = wtforms.StringField('Device ID', [wtforms.validators.InputRequired()])

    seq = wtforms.IntegerField('Sequence number', [wtforms.validators.InputRequired()])

    lat = wtforms.FloatField('Latitude', [wtforms.validators.InputRequired()])
    lng = wtforms.FloatField('Longitude', [wtforms.validators.InputRequired()])
    alt = wtforms.IntegerField('Altitude', [wtforms.validators.InputRequired()])

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
            alt=form.alt.data * 8,

            dist=form.dist.data * 16,
            alt_gain=form.alt_gain.data * 2,
            max_speed=form.max_speed.data / 16,
        )

        db.session.add(probe)
        db.session.flush()

        process_probe(probe)

        db.session.commit()

        # Answers with the backend probe ID, as 8 byte hexadecimal string
        response = {
            form.device.data: {
                'downlinkData': hex(probe.id)[2:].rjust(8 * 2, '0')
            }
        }

        return jsonify(response), 201
    else:
        print(form.errors)
        return 'Bad request', 400

def process_probe(probe: Probe) -> Optional[Activity]:
    if probe.is_idle:
        return None

    activity = None

    # Tries to associate the probe with the latest activity
    latest_activity = Activity.query        \
        .order_by(Activity.id.desc())       \
        .first()

    if latest_activity:
        probes = latest_activity.probes

        if probe.received_at - probes[-1].received_at < INACTIVITY_DELAY:
            # We can associate the probe with an existsing activity.
            activity = latest_activity

            # First we associate all idle activities in between with the activity.
            idle_probes = Probe.query                   \
                .filter(Probe.seq > probes[-1].seq)     \
                .filter(Probe.seq < probe.seq)          \
                .all()
            for p in idle_probes:
                activity.probes.append(p)

    if activity is None:
        # We couldn't associate the latest activity with the probe. Create a new activity.
        activity = Activity()
        db.session.add(activity)
        db.session.flush()

    activity.probes.append(probe)

    return activity

@app.route('/activity/<int:id>')
def activity(id: int):
    act = Activity.query.get_or_404(id)

    return render_template('activity.html', activity=act, maptiler_token=os.environ['MAPTILER_TOKEN'])

@app.route('/activity/<int:id>/gpx')
def activity_gpx(id: int):
    act = Activity.query.get_or_404(id)

    return gpx(act).to_xml(), 200, {'Content-Type': 'text/xml'}

def gpx(activity: Activity):
    gpx = gpxpy.gpx.GPX()

    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)

    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)

    for p in activity.probes:
        point = gpxpy.gpx.GPXTrackPoint(p.lat, p.lng, elevation=p.alt, time=p.received_at)
        gpx_segment.points.append(point)

    return gpx

if __name__ == '__main__':
    db.create_all(app=app)
    app.run()
