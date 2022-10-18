import unittest
from unittest.mock import MagicMock, patch

from robotpy_toolkit_7407.utils.units import deg

import constants
from commands.drivetrain.rotate_in_place import RotateInPlace


class RotateInPlaceTest(unittest.TestCase):

    @patch("time.perf_counter")
    def test_run_until_duration(self, perf_counter_mock: MagicMock) -> None:
        # TESTING STEP 1: Setup test. Create objects, set values.

        # perf_counter_mock come from the patch decorator. This is a MaigcMock object that has been monkey patched (
        # ie: added on at runtime) over time.perf_counter while in this test. A mock is a test object which doesn't
        # do anything (unless we tell it to via side_effect or return_value) but keeps track of what methods were
        # called on it. Later in the test we can assert that the methods we expected to be called were called in the
        # way we expected.
        mock_drivetrain = MagicMock()
        # the perf_counter mock will return these values in order as it is called
        # https://docs.python.org/3/library/unittest.mock.html#unittest.mock.Mock.side_effect
        perf_counter_mock.side_effect = [0, 0.5, 1]
        command = RotateInPlace(
            mock_drivetrain, -12 * deg, 0.8, period=constants.period  # -14
        )
        # Replace the controller with a mock to avoid setting up a more thorough drive train mock
        # that can return poses. This allows self.controller.calculate to be called with the goal
        # from self.subsystem.odometry.getPose() (which because subsystem is a MagicMock, so is the return value
        # of self.subsystem.odometry.getPose() (really is magic ain't it). If we didn't do this
        # the real self.controller.calculate would throw an error when we feed it an object type it doesn't like.
        command.controller = MagicMock()

        # TESTING STEP 2: DO SOMETHING
        # In this case we're going to pretend to be the commands scheduler and call some methods on the
        # commands we are trying to test.
        command.initialize()  # time.perf_counter returns 0
        # TESTING STEP 3: Assert expected results
        # We can inspect variables and call methods in tests.Careful to avoid calling methods with unintended
        # side effects.
        self.assertFalse(command.finished)
        self.assertFalse(command.isFinished())
        perf_counter_mock.assert_called_once()
        command.controller.assert_called_once()
        # We use the assertFalse method instead of `assert not command.finished` because unittest will
        # print a nicer error message. If we were using pytest (which we could. I just picked unittest because
        # it was builtin) we would use assert statements.

        # Now we are going to keep looping through steps 2 and 3 until we have finished what we want to test
        command.execute()  # time.perf_counter returns 0
        self.assertFalse(command.finished)
        self.assertFalse(command.isFinished())
        self.assertEqual(perf_counter_mock.call_count, 2)

        command.execute()  # time.perf_counter returns 1 which is > 0.8
        self.assertTrue(command.isFinished())



