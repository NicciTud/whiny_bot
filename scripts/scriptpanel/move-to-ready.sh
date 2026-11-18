#!/bin/sh

exec rostopic pub -1 /moveit_by_name moveit_by_name/Command "{'group':'arm','target': 'ready'}" 
