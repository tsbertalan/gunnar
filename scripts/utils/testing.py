import unittest
from gunnar.testing import TestGunnar
ts = unittest.TestSuite()
ts.addTest(TestGunnar('test_motorObjectMovement'))
unittest.TextTestRunner().run(ts)

