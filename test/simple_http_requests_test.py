#! /usr/bin/env python

import httplib
import rospy
import unittest

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


if __name__ == '__main__':
    import rostest
    rospy.init_node('simple_http_requests_test')
    rostest.rosrun('async_web_server_cpp', 'simple_http_requests', TestSimpleHttpRequests)
