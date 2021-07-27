#!/usr/bin/python3

# QMesh
# Copyright (C) 2021 Daniel R. Fay

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys, os
import time
import json
import base64
import pika
import subprocess

import qmesh_common
import qmesh_pb2


def run_test(test_name, timeout=30):
    results = ''
    try:
        run_results = subprocess.run(['python', 'unit_tests/' + test_name + '.py', sys.argv[1]], timeout=30,
                                capture_output=True)
        f = open('unit_tests/logs/' + test_name + '.stdout', 'w')
        f.write(str(run_results.stdout, 'utf-8'))
        f.close()
        f = open('unit_tests/logs/' + test_name + '.stderr', 'w')
        f.write(str(run_results.stderr, 'utf-8'))
        f.close()
        if run_results.returncode == 0:
            results = 'Success'
        else:
            results = 'Fail'
    except subprocess.TimeoutExpired:
        results = 'Timeout'
    print('%s: %s' % (test_name, results))


# Start unit testing
print('Starting unit testing...')
run_test('get_status')

