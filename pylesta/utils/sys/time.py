"""
Author: Ikhyeon Cho
Link: https://github.com/Ikhyeon-Cho
File: utils/sys/time.py
"""

from datetime import datetime
from zoneinfo import ZoneInfo


def now(timezone: str = None):
    """
    Get current time in Seoul, Korea.
    """
    if timezone is not None:
        return datetime.now(tz=ZoneInfo(timezone)).strftime('%Y%m%d_%H%M%S')
    else:
        return datetime.now().strftime('%Y%m%d_%H%M%S')


if __name__ == "__main__":

    print(now())
    print(now(timezone='Asia/Seoul'))
