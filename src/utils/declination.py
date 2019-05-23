#!/usr/bin/env python

# Ported from https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Declination/generate
# Run this script with python3!
# To install the igrf module, use python3 -m pip install --user igrf12
# Note that it requires a fortran compiler

'''
generate field tables from IGRF12
'''

import collections
import datetime
import pathlib
import sys

import igrf12
import numpy as np

SAMPLING_RES = 10
SAMPLING_MIN_LAT = -90
SAMPLING_MAX_LAT = 90
SAMPLING_MIN_LON = -180
SAMPLING_MAX_LON = 180

# This is used for flash constrained environments. We limit
# the latitude range to [-60, 60], so the values fit in an int8_t
SAMPLING_COMPACT_MIN_LAT = -60
SAMPLING_COMPACT_MAX_LAT = 60

PREPROCESSOR_SYMBOL = 'NAV_AUTO_MAG_DECLINATION_PRECISE'

Query = collections.namedtuple('Query', ['date', 'res', 'min_lat', 'max_lat', 'min_lon', 'max_lon'])
Result = collections.namedtuple('Result', ['query', 'lats', 'lons', 'declination', 'inclination', 'intensity'])

def write_table(f, name, table, compact):
    '''write one table'''

    if compact:
        format_entry = lambda x: '%d' % round(x)
        table_type = 'int8_t'
    else:
        table_type = 'float'
        format_entry = lambda x: '%.5ff' % x

    num_lat = len(table)
    num_lon = len(table[0])

    f.write("static const %s %s[%u][%u] = {\n" %
                (table_type, name, num_lat, num_lon))
    for i in range(num_lat):
        f.write("    {")
        for j in range(num_lon):
            f.write(format_entry(table[i][j]))
            if j != num_lon - 1:
                f.write(",")
        f.write("}")
        if i != num_lat - 1:
            f.write(",")
        f.write("\n")
    f.write("};\n\n")

def declination_tables(query):
    lats = np.arange(query.min_lat, query.max_lat + query.res, query.res)
    lons = np.arange(query.min_lon, query.max_lon + query.res, query.res)

    num_lat = lats.size
    num_lon = lons.size

    intensity = np.empty((num_lat, num_lon))
    inclination = np.empty((num_lat, num_lon))
    declination = np.empty((num_lat, num_lon))

    for i, lat in enumerate(lats):
        for j, lon in enumerate(lons):
            mag = igrf12.igrf(date, glat=lat, glon=lon, alt_km=0., isv=0, itype=1)
            intensity[i][j] = mag.total / 1e5
            inclination[i][j] = mag.incl
            declination[i][j] = mag.decl

    return Result(query=query, lats=lats, lons=lons,
        declination=declination, inclination=inclination, intensity=intensity)

def generate_constants(f, query):
    f.write('#define SAMPLING_RES\t\t%.5ff\n' % query.res)
    f.write('#define SAMPLING_MIN_LON\t%.5ff\n' % query.min_lon)
    f.write('#define SAMPLING_MAX_LON\t%.5ff\n' % query.max_lon)
    f.write('#define SAMPLING_MIN_LAT\t%.5ff\n' % query.min_lat)
    f.write('#define SAMPLING_MAX_LAT\t%.5ff\n' % query.max_lat)
    f.write('\n')

def generate_tables(f, query, compact):
    result = declination_tables(query)
    write_table(f, 'declination_table', result.declination, compact)

    # We're not using these tables for now
    #if not compact:
    #    write_table(f, 'inclination_table', result.inclination, False)
    #    write_table(f, 'intensity_table', result.intensity, False)

def generate_code(f, date):

    compact_query = Query(date=date, res=SAMPLING_RES,
        min_lat=SAMPLING_COMPACT_MIN_LAT, max_lat=SAMPLING_COMPACT_MAX_LAT,
        min_lon=SAMPLING_MIN_LON, max_lon=SAMPLING_MAX_LON)

    precise_query = Query(date=date, res=SAMPLING_RES,
        min_lat=SAMPLING_MIN_LAT, max_lat=SAMPLING_MAX_LAT,
        min_lon=SAMPLING_MIN_LON, max_lon=SAMPLING_MAX_LON)

    f.write('/* this file is automatically generated by src/utils/declination.py - DO NOT EDIT! */\n\n\n')
    f.write('/* Updated on %s */\n\n\n' % date)
    f.write('#include <stdint.h>\n\n')


    f.write('\n\n#if defined(%s)\n' % PREPROCESSOR_SYMBOL)
    generate_constants(f, precise_query)
    generate_tables(f, precise_query, False)
    # We're not using these tables for now
    # write_table(f, 'inclination_table', inclination_table)
    # write_table(f, 'intensity_table', intensity_table)
    f.write('#else /* !%s */\n' % PREPROCESSOR_SYMBOL)
    generate_constants(f, compact_query)
    generate_tables(f, compact_query, True)
    f.write('#endif\n')

if __name__ == '__main__':

    output = pathlib.PurePath(__file__).parent / '..' / 'main'  / 'navigation' / 'navigation_declination_gen.c'
    date = datetime.datetime.now()

    with open(output, 'w') as f:
        generate_code(f, date)
