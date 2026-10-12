"""Input regression checks without opening a camera or a desktop window."""
import sys
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from boxer_preview import BoxSelection


def drag(selector, start, end):
    selector.mouse(cv2.EVENT_LBUTTONDOWN,*start,0,None)
    selector.mouse(cv2.EVENT_LBUTTONUP,*end,0,None)


def test_confirm_button_finishes_single_box_without_escape_or_keyboard():
    selector=BoxSelection(['bottle'],(480,640,3))
    drag(selector,(300,400),(100,50))
    selector.mouse(cv2.EVENT_LBUTTONDOWN,40,530,0,None)
    assert selector.complete
    assert selector.boxes==[[100,50,200,350]]
    selector.confirm()
    assert len(selector.boxes)==1


def test_repeated_enter_does_not_duplicate_previous_object():
    selector=BoxSelection(['bottle','keyboard'],(480,640,3))
    drag(selector,(10,10),(100,100))
    for _ in range(22):selector.confirm()
    assert len(selector.boxes)==1
    assert not selector.complete
    drag(selector,(150,200),(600,300))
    selector.confirm()
    assert selector.complete


def test_click_is_not_a_box_and_redraw_preserves_confirmed_boxes():
    selector=BoxSelection(['bottle'],(480,640,3))
    drag(selector,(10,10),(10,10))
    selector.confirm()
    assert not selector.boxes
    drag(selector,(10,10),(100,100))
    selector.mouse(cv2.EVENT_LBUTTONDOWN,500,530,0,None)
    assert selector.pending is None
    assert selector.draw(np.zeros((480,640,3),np.uint8)).shape==(560,640,3)
