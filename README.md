# Bike Tracker

This repository contains the code for an Arduino based bicycle GPS tracker and it's Python backend.

## Backend

You can start a development server with:

    DATABASE_URL=<changeme>                             \
    SECRET_KEY=<changeme>                               \
    APP_SETTINGS='backend.config.DevelopmentConfig'     \
    FLASK_ENV=development                               \
    DEVICE_ID=<changeme>                                \
    TIMEZONE=Europe/Brussels                            \
    MAPTILER_TOKEN=<changeme>                           \
    STRAVA_ACCESS_TOKEN=<changeme>                      \
    env/bin/python3 -m backend.app
