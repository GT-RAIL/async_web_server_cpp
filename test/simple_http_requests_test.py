#! /usr/bin/env python

import httplib
import rospy
import unittest
import subprocess
import signal
import time
from catkin.find_in_workspaces import find_in_workspaces

class TestSimpleHttpRequests(unittest.TestCase):
    def setUp(self):
        self.conn = httplib.HTTPConnection("localhost:9849")

    def test_ok(self):
        self.conn.request("GET", "/response/ok")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)

    def test_created(self):
        self.conn.request("GET", "/response/created")
        response = self.conn.getresponse()
        self.assertEqual(201, response.status)

    def test_accepted(self):
        self.conn.request("GET", "/response/accepted")
        response = self.conn.getresponse()
        self.assertEqual(202, response.status)

    def test_forbidden(self):
        self.conn.request("GET", "/response/forbidden")
        response = self.conn.getresponse()
        self.assertEqual(403, response.status)

    def test_not_found(self):
        self.conn.request("GET", "/response/not_found")
        response = self.conn.getresponse()
        self.assertEqual(404, response.status)

    def test_internal_server_error(self):
        self.conn.request("GET", "/response/internal_server_error")
        response = self.conn.getresponse()
        self.assertEqual(500, response.status)

    def test_default_action(self):
        self.conn.request("GET", "/some_random_url12345")
        response = self.conn.getresponse()
        self.assertEqual(404, response.status)

    def test_default_action(self):
        self.conn.request("GET", "/a_static_response")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual("A RESPONSE", response.read())

    def test_http_echo1(self):
        test_content = "hello HELLO"*1000 # make sure to exceed MTU
        self.conn.request("GET", "/http_body_echo", test_content)
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(test_content, response.read())

    def test_http_echo2(self):
        test_content = "THIS is A test"*1000 # make sure to exceed MTU
        self.conn.request("POST", "/http_body_echo", test_content)
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual(test_content, response.read())

    def test_http_path_echo(self):
        self.conn.request("GET", "/http_path_echo/this_is_a_test")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual("/http_path_echo/this_is_a_test", response.read())

    def test_http_query_echo(self):
        self.conn.request("GET", "/http_query_echo?hello=1&b=test&c=10")
        response = self.conn.getresponse()
        self.assertEqual(200, response.status)
        self.assertEqual("b=test\nc=10\nhello=1\n", response.read())

if __name__ == '__main__':
    time.sleep(1) # ensure server is up

    import rostest
    rospy.init_node('simple_http_requests_test')
    rostest.rosrun('async_web_server_cpp', 'simple_http_requests', TestSimpleHttpRequests)
