import unittest
from gunnar.testing import TestGunnar
ts = unittest.TestSuite()
ts.addTest(TestGunnar('test_encodersNoMotors'))
unittest.TextTestRunner().run(ts)

