from types import SimpleNamespace as S
from vlm_nav.go2_manual_recovery import ActionStop


def message(identifier, state):
    return S(status_list=[S(goal_info=S(goal_id=S(uuid=[identifier]*16)),status=state)])


def test_cancel_acceptance_is_not_action_terminal():
    stop=ActionStop()
    stop.observe(message(1,2))
    stop.begin()
    stop.cancelled(S(return_code=0,goals_canceling=[]))
    assert not stop.terminal
    stop.observe(message(1,5))
    assert stop.terminal


def test_lost_goal_does_not_become_terminal_from_empty_status():
    stop=ActionStop()
    stop.observe(message(1,2))
    stop.begin()
    stop.cancelled(S(return_code=0,goals_canceling=[]))
    stop.observe(S(status_list=[]))
    assert not stop.terminal


def test_no_goal_requires_successful_cancel_response():
    stop=ActionStop()
    stop.begin()
    assert not stop.terminal
    stop.cancelled(S(return_code=1,goals_canceling=[]))
    assert not stop.terminal
    stop.cancelled(S(return_code=0,goals_canceling=[]))
    assert stop.terminal


def test_terminal_before_cancel_reply_remains_terminal():
    stop=ActionStop()
    stop.observe(message(1,2))
    stop.begin()
    stop.observe(message(1,5))
    stop.cancelled(S(return_code=0,goals_canceling=[S(goal_id=S(uuid=[1]*16))]))
    assert stop.terminal
