## Parakeet Web Service

This python script runs a HTTP service on an specified port which then receives the data uploaded by the Parakeet

It provides a JSON service which can then be utilized by the 'wifi wixel' ip mode supported by the xDrip Android application.

In testing this has been run on a raspberry pi which exists on a local wifi-network but also with dynamic dns and router port mapping to allow data to come in via the GSM network.

Future expansion of this script could include writing to a mongo database or other similar service.

**This script is now only really used in an "advanced" configuration. It is much easier to use the Google App Engine version. For details see https://jamorham.github.io/**
