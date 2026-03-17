#!/usr/bin/env python3
import re
import difflib
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def normalize_thai(text: str) -> str:
    t = (text or "").strip()
    t = t.replace("ๆ", "")
    t = re.sub(r"\s+", " ", t)
    return t

ALIAS_REPLACEMENTS = [
    (r"(ปลดล็อค|ปลดล๊อค|ปลดล็อก|\bunlock\b)", "ปลดล็อก"),
    (r"(ล็อค|ล๊อค|ล็อก|\block\b)", "ล็อก"),
    (r"(รีเซ็ต|รีเซท|\breset\b|เริ่มใหม่|ล้างค่า|ล้างระบบ|รีสตาร์ทคำสั่ง|session\s*reset)", "รีเซ็ตเซสชัน"),
    (r"(ตั้งค่าใหม่|ตั้งค่า\s*ใหม่|รี\s*เซ็ต\s*ค่า|รีเซ็ต\s*ค่า)", "รีเซ็ตเซสชัน"),

    (r"อะ\s*ไร", "อะไร"),
    (r"องศ(?=\s|$)", "องศา"),
    (r"องสา", "องศา"),
    (r"องศาา+", "องศา"),

    (r"(มุมมอง|มุมมองที่|วิว|view)\s*บน\b", "มุมมองบน"),
    (r"(มุมมอง|มุมมองที่|วิว|view)\s*ข้าง\b", "มุมมองข้าง"),
    (r"(มุม|มุมมอง|วิว|view)\s*ด้าน\s*บน\b", "มุมมองบน"),
    (r"(มุม|มุมมอง|วิว|view)\s*ด้าน\s*ข้าง\b", "มุมมองข้าง"),
    (r"(มุม|มุมมอง|วิว|view)\s*ด้านบน\b", "มุมมองบน"),
    (r"(มุม|มุมมอง|วิว|view)\s*ด้านข้าง\b", "มุมมองข้าง"),

    (r"มุมุ\s*บน\b", "มุมมองบน"),
    (r"มุมุบน\b", "มุมมองบน"),

    (r"(^|\s)มุม\s*บน(\s|$)", " มุมมองบน "),
    (r"(^|\s)มุม\s*ข้าง(\s|$)", " มุมมองข้าง "),

    (r"\btop[\s\-]*view\b", "มุมมองบน"),
    (r"\btop[\s\-]*approach\b", "มุมมองบน"),
    (r"\bpop\b", "top"),
    (r"(ท็อป|ทอป)\s*(แอพโพรช|แอพโปรช|แอพโพรส)\b", "มุมมองบน"),
    (r"\bside[\s\-]*view\b", "มุมมองข้าง"),
    (r"\bไซด์\s*(แอพโพรช|แอพโปรช|แอพโพรส)\b", "มุมมองข้าง"),
    (r"\bซาย\s*(แอพโพรช|แอพโปรช|แอพโพรส)\b", "มุมมองข้าง"),
    (r"\b(cide|sibe|sipe)\b", "side"),
    (r"\bsid\b", "side"),
    (r"\bsied\b", "side"),
    (r"\bไซ\b", "side"),
    (r"(ท็อป|ทอป)\s*วิว\b", "มุมมองบน"),
    (r"(ท็อปวิว|ทอปวิว)\b", "มุมมองบน"),
    (r"(ไซด์|ซายด์|ซาย)\s*วิว\b", "มุมมองข้าง"),
    (r"(ไซด์วิว|ซายด์วิว|ซายวิว)\b", "มุมมองข้าง"),

    (r"ระบบ\s*มุม\b", "ระบุมุม"),
    (r"ระบุ\s*มุม\b", "ระบุมุม"),
    (r"ระบู\s*มุม\b", "ระบุมุม"),
    (r"ระบุมล\b", "ระบุมุม"),
    (r"ระบุหมุน\b", "ระบุมุม"),

    (r"ระบบ\s*ระยะ\b", "ระบุระยะ"),
    (r"ระบุ\s*ระยะ\b", "ระบุระยะ"),
    (r"ระบู\s*ระยะ\b", "ระบุระยะ"),
    (r"ระบุ\s*จำนวน\b", "ระบุระยะ"),

    (r"เลื่อน\s*ขึ้น\b", "เลื่อนขึ้น"),
    (r"เลื่อน\s*ลง\b", "เลื่อนลง"),
    (r"หมุน\s*ซ้าย\b", "หมุนซ้าย"),
    (r"หมุน\s*ขวา\b", "หมุนขวา"),
    (r"ไป\s*ที่\b", "ไปที่"),

    (r"หมุน\s*ไป\s*ทาง\s*ซ้าย\b", "หมุนซ้าย"),
    (r"หมุน\s*ไป\s*ทาง\s*ขวา\b", "หมุนขวา"),

    (r"\bturn\s*left\b", "หมุนซ้าย"),
    (r"\bturn\s*right\b", "หมุนขวา"),
    (r"\brotate\s*(to\s*)?the\s*left\b", "หมุนซ้าย"),
    (r"\brotate\s*(to\s*)?the\s*right\b", "หมุนขวา"),
    (r"\brotate\s*left\b", "หมุนซ้าย"),
    (r"\brotate\s*right\b", "หมุนขวา"),

    (r"(เทิร์น|เทิน|เทิรน์)\s*(เล็บ|เลฟ|เล็ฟ|เลฟท์|เล็ฟท์)\b", "หมุนซ้าย"),
    (r"(เทิร์น|เทิน|เทิรน์)\s*(ไรท์|ไรท|ร้าย|ไลท์|ไลท)\b", "หมุนขวา"),

    (r"ไป\s*ทาง\s*ซ้าย\b", "ไปซ้าย"),
    (r"ไป\s*ทาง\s*ขวา\b", "ไปขวา"),
    (r"ไป\s*ด้าน\s*หน้า\b", "ไปหน้า"),
    (r"ไป\s*ด้าน\s*หลัง\b", "ไปหลัง"),

    (r"ถอย\s*หลัง\b", "ไปหลัง"),
    (r"(^|\s)ถอย(\s|$)", " ไปหลัง "),

    (r"(ขยับ|เลื่อน|ไป)\s*ซ้าย\b", "ไปซ้าย"),
    (r"(ขยับ|เลื่อน|ไป)\s*ขวา\b", "ไปขวา"),
    (r"(ขยับ|เลื่อน|ไป)\s*หน้า\b", "ไปหน้า"),
    (r"(ขยับ|เลื่อน|ไป)\s*หลัง\b", "ไปหลัง"),
    (r"(ขยับ|เลื่อน|ไป)\s*ขึ้น\b", "ขยับขึ้น"),
    (r"(ขยับ|เลื่อน|ไป)\s*ลง\b", "ขยับลง"),

    (r"เลื่อน\s*ข้าง\s*ซ้าย\b", "ไปซ้าย"),
    (r"เลื่อน\s*ข้าง\s*ขวา\b", "ไปขวา"),
    (r"เลื่อนข้างซ้าย\b", "ไปซ้าย"),
    (r"เลื่อนข้างขวา\b", "ไปขวา"),
    (r"(?:^|\s)เลื่อน\s*ข้าง\s*$", " ไปซ้าย "),
    (r"(?:^|\s)เลื่อนข้าง\s*$", " ไปซ้าย "),

    (r"\bgo\s*left\b", "ไปซ้าย"),
    (r"\bmove\s*left\b", "ไปซ้าย"),
    (r"\bslide\s*left\b", "ไปซ้าย"),
    (r"\bgo\s*right\b", "ไปขวา"),
    (r"\bmove\s*right\b", "ไปขวา"),
    (r"\bslide\s*right\b", "ไปขวา"),

    (r"(^|\s)ด้าน\s*ซ้าย(\s|$)", " ซ้าย "),
    (r"(^|\s)ด้าน\s*ขวา(\s|$)", " ขวา "),

    (r"(^|\s)สาย(\s|$)", " ซ้าย "),
    (r"\bleft\b", "ซ้าย"),
    (r"(^|\s)เลฟ(\s|$)", " ซ้าย "),
    (r"(^|\s)เล็ฟ(\s|$)", " ซ้าย "),
    (r"(^|\s)เลฝ(\s|$)", " ซ้าย "),
    (r"(^|\s)เลฟท์(\s|$)", " ซ้าย "),
    (r"(^|\s)เลท(\s|$)", " ซ้าย "),
    (r"มูม\s*เลฟท์\b", "ซ้าย"),
    (r"มูม\s*เลฟ\b", "ซ้าย"),
    (r"มูม\s*เล็ฟ\b", "ซ้าย"),
    (r"มูม\s*เลท\b", "ซ้าย"),
    (r"มวกเหล็ก", "ซ้าย"),
    (r"มุขเด็ด", "ซ้าย"),
    (r"มุกเด็ด", "ซ้าย"),
    (r"หมูเล็บ", "ไปซ้าย"),

    (r"\bright\b", "ขวา"),
    (r"(^|\s)ไรท์(\s|$)", " ขวา "),
    (r"(^|\s)ไรท(\s|$)", " ขวา "),
    (r"(^|\s)ไร(\s|$)", " ขวา "),
    (r"(^|\s)ราย(\s|$)", " ขวา "),
    (r"(^|\s)ร้าย(\s|$)", " ขวา "),
    (r"มูม\s*ไรท์\b", "ขวา"),
    (r"มูม\s*ไรท\b", "ขวา"),
    (r"มูม\s*ไร(\s|$)", "ขวา"),
    (r"มูม\s*ราย\b", "ขวา"),
    (r"\bmove\s*like\b", "ขวา"),
    (r"\bmove\s*light\b", "ขวา"),
    (r"\bmove\s*rite\b", "ขวา"),
    (r"มูฟ\s*ไลค์\b", "ขวา"),
    (r"มูม\s*ไลค์\b", "ขวา"),
    (r"มูม\s*ลาย\b", "ขวา"),
    (r"\blight\b", "right"),
    (r"\blike\b", "right"),
    (r"\blite\b", "right"),

    (r"(move|มูม|มูฟ)\s*(up|อัพ|อัป|อัฟ|เอ้ป|แอป)\b", "ขยับขึ้น"),
    (r"มูมอัพ\b", "ขยับขึ้น"),
    (r"มูฟอัพ\b", "ขยับขึ้น"),
    (r"\bmove\s*up\b", "ขยับขึ้น"),

    (r"(move|มูม|มูฟ|มุก)\s*(down|ดาว|ดาวน์|ด่าน|ดวน์|เด้า)\b", "ขยับลง"),
    (r"มุกดาว\b", "ขยับลง"),
    (r"มูมดาว\b", "ขยับลง"),
    (r"มูมดาวน์\b", "ขยับลง"),
    (r"มูฟดาวน์\b", "ขยับลง"),
    (r"\bmove\s*down\b", "ขยับลง"),

    (r"ซ้าย\s*นิด\b", "ซ้ายนิด"),
    (r"ขวา\s*นิด\b", "ขวานิด"),
    (r"ขึ้น\s*นิด\b", "ขึ้นนิด"),
    (r"ลง\s*นิด\b", "ลงนิด"),

    (r"โพชิชั่น", "ตำแหน่ง"),
    (r"โพซิชั่น", "ตำแหน่ง"),
    (r"โพสิชั่น", "ตำแหน่ง"),
    (r"โพซิชัน", "ตำแหน่ง"),
    (r"โพสิชัน", "ตำแหน่ง"),
    (r"\bposition\b", "ตำแหน่ง"),
    (r"\bpos\b", "ตำแหน่ง"),
    (r"(ตำแหน่ง)([1-5])\b", r"ตำแหน่ง \2"),
    (r"(ตำแหน่ง)(one|two|three|four|five)\b", r"ตำแหน่ง \2"),
    (r"(ตำแหน่ง)(วัน|ทู|ทรี|โฟร์|ไฟว์)\b", r"ตำแหน่ง \2"),
    (r"(มุมมองบน)(ตำแหน่ง)", r"\1 \2"),
    (r"(มุมมองข้าง)(ตำแหน่ง)", r"\1 \2"),
    (r"(top[\s\-]*view)(position|pos)\b", r"\1 \2"),
    (r"(side[\s\-]*view)(position|pos)\b", r"\1 \2"),

    # --- misrecognitions ---
    (r"\bprotection\b", "ตำแหน่ง"),
    (r"\baudition\b", "ตำแหน่ง"),
    (r"\bfour\s*season(?:s)?\b", "ตำแหน่ง"),   
    (r"\bfour[\s\-]*season(?:s)?\b", "ตำแหน่ง"),

    # misrecognitions for number five 
    (r"\bfight\b", "five"),
    (r"\bfife\b", "five"),
    (r"\bfine\b", "five"),
    (r"\bfire\b", "five"),
    (r"\bpipe\b", "five"),

    # Thai 
    (r"ไฟว์\b", "five"),

    # IMPORTAN
    (r"(ตำแหน่ง)\s*ไฟ\b", r"\1 five"),

    (r"\bback\s*home\b", "กลับบ้าน"),
    (r"กลับ\s*home\b", "กลับบ้าน"),
    (r"กลับ\s*บ้าน\b", "กลับบ้าน"),
    (r"กลับ\s*โฮม\b", "กลับบ้าน"),

    (r"\bpick\s*up\b", "หยิบ"),
    (r"\bgrab\s*it\b", "หยิบ"),
    (r"\bpick\s*it\b", "หยิบ"),
    (r"พิค\s*อัพ\b", "หยิบ"),
    (r"พิก\s*อัพ\b", "หยิบ"),
    (r"ปิ๊ก\s*อัพ\b", "หยิบ"),
    (r"พิคอัพ\b", "หยิบ"),
    (r"แกร็บ\s*อิท\b", "หยิบ"),
    (r"แกรบ\s*อิท\b", "หยิบ"),
    (r"ปิค\s*อัพ\b", "หยิบ"),
    (r"ปิคอัพ\b", "หยิบ"),
    (r"ปิก\s*อัพ\b", "หยิบ"),
    (r"ปิกอัพ\b", "หยิบ"),

    (r"\bput\s*down\b", "วาง"),
    (r"\bplace\s*down\b", "วาง"),
    (r"\blet\s*go\b", "วาง"),
    (r"พุท\s*ดาวน์\b", "วาง"),
    (r"พุทดาวน์\b", "วาง"),
    (r"เพลส\s*ดาวน์\b", "วาง"),
    (r"เพลสดาวน์\b", "วาง"),
    (r"เล็ท\s*โก\b", "วาง"),
    (r"เลทโก\b", "วาง"),
    (r"\bfoot\s*down\b", "วาง"),
    (r"พุธ\s*ดาว\b", "วาง"),
    (r"พุธดาว\b", "วาง"),

    (r"\bspeed[\s:\-_]*normal\b", "สปีดปกติ"),
    (r"\bspeed[\s:\-_]*norm\b", "สปีดปกติ"),
    (r"\bspeed[\s:\-_]*fast\b", "สปีดเร็ว"),
    (r"\bspeed[\s:\-_]*slow\b", "สปีดช้า"),
    (r"\bspeednormal\b", "สปีดปกติ"),
    (r"\bspeedfast\b", "สปีดเร็ว"),
    (r"\bspeedslow\b", "สปีดช้า"),
    (r"(สปีด|ความเร็ว)\s*(นอมอล|นอร์มอล|นอร์ม|normal|norm)\b", "สปีดปกติ"),
    (r"(สปีด|ความเร็ว)\s*(ฟาส|แฟส|fast)\b", "สปีดเร็ว"),
    (r"(สปีด|ความเร็ว)\s*(สโล|สโลว์|slow)\b", "สปีดช้า"),
    (r"\bspeed\s*up\b", "สปีดเร็ว"),
    (r"\bspeed\s*down\b", "สปีดช้า"),
    (r"\bincrease\s*speed\b", "สปีดเร็ว"),
    (r"\bdecrease\s*speed\b", "สปีดช้า"),
    (r"\bgo\s*fast\b", "สปีดเร็ว"),
    (r"\bgo\s*slow\b", "สปีดช้า"),
    (r"\bset\s*speed\s*to\s*fast\b", "สปีดเร็ว"),
    (r"\bset\s*speed\s*to\s*normal\b", "สปีดปกติ"),
    (r"\bset\s*speed\s*to\s*slow\b", "สปีดช้า"),
    (r"\bplease\s*speed\s*up\b", "สปีดเร็ว"),
    (r"\bplease\s*slow\s*down\b", "สปีดช้า"),
    (r"(สปีด|ความเร็ว)\s*(ช้า|slow)\b", "สปีดช้า"),
    (r"(สปีด|ความเร็ว)\s*(เร็ว|ไว|fast)\b", "สปีดเร็ว"),
    (r"(สปีด|ความเร็ว)\s*(ปกติ|ธรรมดา|normal)\b", "สปีดปกติ"),

    (r"ข้อมือ\s*สาม\b", "w3"),
    (r"ข้อมือ3\b", "w3"),
    (r"ริสท์\s*สาม\b", "w3"),
    (r"wrist\s*3\b", "w3"),
    (r"wrist3\b", "w3"),
    (r"(ปลาย\s*แขน)\s*(สาม|3)\b", "w3"),
    (r"(ปลาย\s*แขน)\b", "w3"),
    (r"\bแขน\s*(สาม|3)\b", "w3"),
    (r"\bแขน3\b", "w3"),
    (r"\bend\s*effector\b", "w3"),
    (r"\bendeffector\b", "w3"),
    (r"\btool0\b", "w3"),
    (r"\btool\b", "w3"),

    # --- W3 / End-effector / Tool (Thai misrecognitions) ---
    (r"ตูน(?=\S)", "w3 "),
    (r"(ทูล|ทูน|ตูน|ทูน|ทู|ทุล)\b", "w3"),
    (r"(ทูล\s*ซีโร่|ทูล\s*ศูนย์|tool\s*zero)\b", "w3"),
    (r"(ทู\s*ล|ทู\s*น)\b", "w3"),
    (r"(ปลาย\s*แขน|ปลายแข็ง|ปลายแหน|ปลายแขนกล)\b", "w3"),

]

NUM_WORDS = {
    "1": 1, "หนึ่ง": 1, "one": 1, "วัน": 1,
    "2": 2, "สอง": 2, "two": 2, "to": 2, "ทู": 2,
    "3": 3, "สาม": 3, "three": 3, "ทรี": 3,
    "4": 4, "สี่": 4, "four": 4, "โฟร์": 4, "โฟ": 4,
    "5": 5, "ห้า": 5, "five": 5,
    "ไฟ": 5, "ไฟว์": 5, "fife": 5, "vive": 5, "fight": 5, "fine": 5, "pipe": 5,
}

def normalize_for_nlu(text: str) -> str:
    """Lowercase + apply alias replacements + collapse whitespace."""
    t = normalize_thai(text).lower()
    for pat, rep in ALIAS_REPLACEMENTS:
        t = re.sub(pat, rep, t)
    t = re.sub(r"\s+", " ", t).strip()
    return t

def contains_fuzzy(keyword: str, text: str, th: float = 0.78) -> bool:
    """Check if any token in text approximately matches the keyword (SequenceMatcher threshold)."""
    toks = re.split(r"\s+", text)
    for tk in toks:
        if difflib.SequenceMatcher(None, keyword, tk).ratio() >= th:
            return True
    return False

_TH_DIGITS = {
    "ศูนย์": 0,
    "หนึ่ง": 1, "เอ็ด": 1,
    "สอง": 2, "ยี่": 2,
    "สาม": 3,
    "สี่": 4,
    "ห้า": 5,
    "หก": 6,
    "เจ็ด": 7,
    "แปด": 8,
    "เก้า": 9,
}

def fuzzy_phrase_contains(text: str, phrase_tokens: list[str], threshold: float = 0.75) -> bool:
    """
    Check if text approximately contains a sequence of tokens (order matters).
    Example: ["turn", "left"]
    """
    toks = re.findall(r"[a-zก-๙0-9]+", text.lower())

    for i in range(len(toks) - len(phrase_tokens) + 1):
        match_all = True
        for j, ref in enumerate(phrase_tokens):
            sim = difflib.SequenceMatcher(None, toks[i + j], ref).ratio()
            if sim < threshold:
                match_all = False
                break
        if match_all:
            return True
    return False

def _parse_thai_under_100(s: str) -> Optional[int]:
    """Parse Thai number words under 100 (supports 'สิบ' patterns)."""
    s = (s or "").strip()
    if not s:
        return None

    if s in _TH_DIGITS and s != "ยี่":
        return _TH_DIGITS[s]

    if "สิบ" in s:
        parts = s.split("สิบ", 1)
        tens_w = parts[0].strip()
        ones_w = parts[1].strip()

        if tens_w == "":
            tens = 1
        elif tens_w == "ยี่":
            tens = 2
        elif tens_w in _TH_DIGITS:
            tens = _TH_DIGITS[tens_w]
        else:
            return None

        ones = 0
        if ones_w:
            if ones_w in _TH_DIGITS:
                ones = _TH_DIGITS[ones_w]
            else:
                return None
        return tens * 10 + ones

    if s in _TH_DIGITS:
        return _TH_DIGITS[s]
    return None


def parse_thai_number_0_999(s: str) -> Optional[int]:
    """Parse Thai number words 0..999 (supports '<digit>ร้อย' + under-100)."""
    s = (s or "").strip()
    if not s:
        return None

    if "ร้อย" in s:
        h_part, rest = s.split("ร้อย", 1)
        h_part = h_part.strip()
        rest = rest.strip()

        if h_part in _TH_DIGITS and h_part != "ยี่":
            hundreds = _TH_DIGITS[h_part]
        else:
            return None

        if rest == "":
            return hundreds * 100

        under100 = _parse_thai_under_100(rest)
        if under100 is None:
            if rest in _TH_DIGITS and rest != "ยี่":
                under100 = _TH_DIGITS[rest]
            else:
                return None
        return hundreds * 100 + under100

    return _parse_thai_under_100(s)


def parse_degrees(text: str) -> Optional[float]:
    """Extract degree value from text (supports numeric and Thai words when degree unit is present)."""
    t = text.lower()

    m = re.search(r"(\d+(?:\.\d+)?)\s*(องศา|deg|degree|degrees)(?=\D|$)", t)
    if m:
        try:
            return float(m.group(1))
        except Exception:
            return None

    if ("องศา" in t) or ("degree" in t) or ("deg" in t):
        m2 = re.search(r"([ก-๙]+)\s*(องศา|deg|degree|degrees)", t)
        if m2:
            w = m2.group(1).strip()
            n = parse_thai_number_0_999(w)
            if n is not None:
                return float(n)

        thai_tokens = re.findall(r"[ก-๙]+", t)
        for tok in thai_tokens:
            n = parse_thai_number_0_999(tok)
            if n is not None:
                return float(n)

    return None


def parse_distance(text: str) -> Optional[float]:
    """Extract distance from text and return value in meters (cm/mm/m supported)."""
    t = text.lower()
    m = re.search(r"(\d+(?:\.\d+)?)\s*(cm|เซน|เซนติ|มม|mm|m|เมตร)?(?=\D|$)", t)
    if not m:
        return None
    try:
        val = float(m.group(1))
    except Exception:
        return None
    unit = (m.group(2) or "").strip()
    if unit in ["cm", "เซน", "เซนติ"]:
        return val / 100.0
    if unit in ["มม", "mm"]:
        return val / 1000.0
    if unit in ["m", "เมตร"]:
        return val
    return val


def parse_position(text: str) -> Tuple[str, Optional[int]]:
    """Parse position intent: POS_1..POS_5, or ask for which position."""
    t = text.lower()

    m = re.search(r"ตำแหน่ง(?:ที่)?\s*([1-5])(?=\D|$)", t)
    if m:
        return ("pos", int(m.group(1)))

    m2 = re.search(r"ตำแหน่ง(?:ที่)?\s*([a-zก-๙]+)(?=\D|$)", t)
    if m2:
        w = m2.group(1).strip()
        if w in NUM_WORDS and 1 <= NUM_WORDS[w] <= 5:
            return ("pos", NUM_WORDS[w])

    tt = t.strip()
    if tt in NUM_WORDS and 1 <= NUM_WORDS[tt] <= 5:
        return ("pos", NUM_WORDS[tt])

    if re.fullmatch(r"(ตำแหน่ง|ตำแหน่งที่)", t):
        return ("pos_ask", None)
    if re.search(r"(ไป|ไปที่)\s*ตำแหน่ง(ที่)?\s*$", t):
        return ("pos_ask", None)

    return ("other", None)


def parse_pick_place(text: str) -> str:
    """Parse pick/place intent, ignoring negative commands (ไม่/อย่า/ห้าม)."""
    t = text.lower()
    neg = re.search(r"(ไม่|อย่า|ห้าม)", t) is not None
    if re.search(r"(หยิบ|จับ|คีบ|หนีบ|pick|grasp|grab)", t) and not neg:
        return "pick"
    if re.search(r"(วาง|ปล่อย|place|release|drop)", t) and not neg:
        return "place"
    return "other"


def parse_lock_unlock(text: str) -> str:
    """Parse lock/unlock intent, ignoring negative commands (ไม่/อย่า/ห้าม)."""
    t = text.lower()
    neg = re.search(r"(ไม่|อย่า|ห้าม)", t) is not None
    if re.search(r"(ปลดล็อก|\bunlock\b)", t) and not neg:
        return "unlock"
    if re.search(r"(ล็อก|\block\b)", t) and not neg:
        return "lock"
    return "other"


def parse_session_reset(text: str) -> bool:
    """Return True if user requests session reset (reset/เริ่มใหม่/ล้างค่า/session reset)."""
    t = text.lower()
    return re.search(r"(รีเซ็ตเซสชัน|รีเซ็ต|รีเซท|\breset\b|เริ่มใหม่|ล้างค่า|ล้างระบบ|session\s*reset)", t) is not None


def parse_speed(text: str) -> Tuple[str, Optional[str]]:
    """Parse speed intent: slow/normal/fast."""
    t = text.lower()
    if re.search(r"(สปีด|ความเร็ว)", t) or re.fullmatch(r"(เร็ว|ช้า|ปกติ|ธรรมดา|ไว|เร็วขึ้น|ช้าลง|normal|fast|slow|faster|slower)", t):
        if re.search(r"(ช้า|slow|slower|ช้าลง)", t):
            return ("speed", "slow")
        if re.search(r"(ปกติ|ธรรมดา|normal)", t):
            return ("speed", "normal")
        if re.search(r"(เร็ว|ไว|fast|faster|เร็วขึ้น)", t):
            return ("speed", "fast")
    return ("other", None)

def parse_view_kind_loose(text: str) -> Optional[str]:
    t = (text or "").lower()

    if "มุมมองบน" in t:
        return "top"
    if "มุมมองข้าง" in t:
        return "side"

    return None



def parse_view(text: str) -> Tuple[str, Optional[str], Optional[int]]:
    t = (text or "").lower()
    if re.search(r"(rotate|turn|หมุน)", t):
        return ("other", None, None)
    
    if re.search(r"(ขยับ|เลื่อน|move|ขึ้น|ลง|ซ้าย|ขวา|หน้า|หลัง)", t):
        return ("other", None, None)
    tokens = re.findall(r"[a-zก-๙0-9]+", t)

    # -------------------------------------------------
    # 1️⃣ หาเลข 1–5 ก่อน
    # -------------------------------------------------
    num = None
    for tok in tokens:
        if tok.isdigit() and 1 <= int(tok) <= 5:
            num = int(tok)
            break
        if tok in NUM_WORDS and 1 <= NUM_WORDS[tok] <= 5:
            num = NUM_WORDS[tok]
            break

    # -------------------------------------------------
    # 2️⃣ keyword reference sets
    # -------------------------------------------------

    TOP_KEYWORDS = [
        "top", "pop",
        "ท็อป", "ทอป",
        "มุมบน", "ด้านบน", "บน"
    ]

    SIDE_KEYWORDS = [
        "side",
        "ไซด์", "ซายด์", "ซาย", "ไซ",
        "มุมข้าง", "ด้านข้าง", "ข้าง"
    ]

    BLOCK_TOP_FALSE = ["post", "position", "pos"]

    # -------------------------------------------------
    # 3️⃣ fuzzy matcher
    # -------------------------------------------------

    def fuzzy_match(tok: str, keywords: list[str], threshold: float = 0.75) -> bool:
        for kw in keywords:
            if tok == kw:
                return True
            if difflib.SequenceMatcher(None, tok, kw).ratio() >= threshold:
                return True
        return False

    def is_top(tok: str) -> bool:
        if tok in BLOCK_TOP_FALSE:
            return False
        return fuzzy_match(tok, TOP_KEYWORDS, 0.75)

    def is_side(tok: str) -> bool:
        return fuzzy_match(tok, SIDE_KEYWORDS, 0.27)

    has_top = any(is_top(tok) for tok in tokens)
    has_side = any(is_side(tok) for tok in tokens)

    # -------------------------------------------------
    # 4️⃣ ตัดสินใจ
    # -------------------------------------------------

    if num is not None:
        if has_top and not has_side:
            return ("view", "top", num)
        if has_side and not has_top:
            return ("view", "side", num)

        # ถ้าทั้งสองอย่างโผล่ → เลือกอันที่ชัดกว่า
        if has_top:
            return ("view", "top", num)

    if has_top and num is None:
        return ("view_ask_pos", "top", None)

    if has_side and num is None:
        return ("view_ask_pos", "side", None)

    return ("other", None, None)

def parse_number_loose(text: str) -> Optional[float]:
    t = (text or "").lower()
    m = re.search(r"(\d+(?:\.\d+)?)\b", t)
    if m:
        try:
            return float(m.group(1))
        except Exception:
            return None
    return None

def parse_rotate(text: str) -> Tuple[str, Optional[str], Optional[float]]:
    t = (text or "").lower()

   # 🚫 ถ้ามีหน่วยระยะ → ห้ามเข้า rotate
    if re.search(r"(cm|เซน|เซนติ|เมตร|มม|mm|\bm\b)", t):
        return ("other", None, None)

    # ✅ trigger หมุน
    if not (re.search(r"(หมุน|rotate|turn)", t) or "ร" in t):
        return ("other", None, None)

    # -------------------------
    # 2️⃣ หาองศา
    # -------------------------
    deg = parse_degrees(t)
    if deg is None:
        deg = parse_number_loose(t)

    # -------------------------
    # 3️⃣ ตรวจ direction ชัดเจนก่อน
    # -------------------------
    if re.search(r"(ซ้าย|left|ทวน)", t):
        return ("rotate", "left", deg)

    if re.search(r"(ขวา|right|ตาม)", t):
        return ("rotate", "right", deg)

    # -------------------------
    # 4️⃣ ใช้สระเป็นตัวตัดสิน
    # -------------------------
    if "เ" in t and "ไ" not in t:
        return ("rotate", "left", deg)

    if "ไ" in t and "เ" not in t:
        return ("rotate", "right", deg)

    # ถ้ามีทั้งสอง หรือไม่มีเลย → ถามใหม่
    return ("rotate_ask_dir", None, None)
        


def parse_w3_rotate(text: str) -> Tuple[str, Optional[str], Optional[float]]:
    t = (text or "").lower()

    w3_pat = r"(w3|wrist\s*3|wrist3|ข้อมือสาม|ข้อมือ3|ปลายแขน|ปลาย\s*แขน|แขน3|\btool0\b|\btool\b|end[\s\-]*effector|\bendeffector\b)"

    if not re.search(w3_pat, t):
        return ("other", None, None)

    left = re.search(r"(ซ้าย|left|ทวน)", t)
    right = re.search(r"(ขวา|right|ตาม)", t)

    t_no_w3 = t
    t_no_w3 = re.sub(r"\bw3\b", " ", t_no_w3)
    t_no_w3 = re.sub(r"wrist\s*3|wrist3", " ", t_no_w3)
    t_no_w3 = re.sub(w3_pat, " ", t_no_w3)
    t_no_w3 = re.sub(r"\s+", " ", t_no_w3).strip()

    deg = parse_degrees(t_no_w3)
    if deg is None:
        deg = parse_number_loose(t_no_w3)

    if ("ระบุมุม" in t) or contains_fuzzy("ระบุมุม", t):
        if left:
            return ("w3_rotate_ask_deg", "left", None)
        if right:
            return ("w3_rotate_ask_deg", "right", None)
        return ("w3_rotate_ask_dir", None, None)

    if left:
        return ("w3_rotate", "left", deg)
    if right:
        return ("w3_rotate", "right", deg)

    return ("w3_rotate_ask_dir", None, None)

def parse_scroll(text: str) -> Tuple[str, Optional[str], Optional[float]]:
    t = text.lower()
    if not re.search(r"(เลื่อน|scroll)", t):
        return ("other", None, None)

    up = re.search(r"(ขึ้น|up)", t) or re.search(r"\bup\b", t)
    down = re.search(r"(ลง|ล่าง|down)", t)
    deg = None
    if "องศา" in t:
        deg = parse_degrees(t)

    if ("ระบุมุม" in t) or contains_fuzzy("ระบุมุม", t):
        if up:
            return ("scroll_ask_deg", "up", None)
        if down:
            return ("scroll_ask_deg", "down", None)
        return ("scroll_ask_dir", None, None)

    if up:
        return ("scroll", "up", deg) if deg is not None else ("scroll_simple", "up", None)
    if down:
        return ("scroll", "down", deg) if deg is not None else ("scroll_simple", "down", None)

    return ("scroll_ask_dir", None, None)

def parse_move(text: str) -> Tuple[str, Optional[str], Optional[float]]:
    """Parse move intent: direction + optional distance; support asking for direction/distance.
       NEW: Support compact form like moveup / moleft / movrigh
       Rule: if contains 'm' and 'o' => treat as move
             direction by first letter u/d/l/r
    """
    t = text.lower().strip()

    # -------------------------------------------------
    # 🔥 NEW: Compact move detection (mo + first letter)
    # Example: moveup / mo u / moleft / movrigh
    # -------------------------------------------------
    compact = re.sub(r"\s+", "", t)

    if (("m" in compact and "o" in compact) or ("ห" in compact)):
        mo_index = compact.find("mo")
        if mo_index != -1 and len(compact) > mo_index + 2:
            next_char = compact[mo_index + 2]

            dir_map = {
                "u": "up",
                "d": "down",
                "l": "left",
                "r": "right",
                "v": "left",
                "เ": "left",  
                "ไ": "right",  
            }

            if next_char in dir_map:
                return ("move_simple", dir_map[next_char], None)

        # support lift → up
        if "lift" in compact:
            return ("move_simple", "up", None)

    # -------------------------------------------------
    # กันชนคำสั่งตำแหน่ง
    # -------------------------------------------------
    if re.search(r"(ไป|ไปที่)\s*ตำแหน่ง", t) or \
       re.search(r"ไป\s*ตำแหน่ง", t) or \
       re.search(r"ไปตำแหน่ง", t):
        return ("other", None, None)

    has_move_word = re.search(r"(ไป|ขยับ|เลื่อน|move|jog)", t) is not None

    # -------------------------------------------------
    # ถ้าไม่มีคำว่า move แต่พิมพ์ทิศอย่างเดียว
    # -------------------------------------------------
    if not has_move_word:
        if re.fullmatch(r"(ซ้าย|ขวา|หน้า|หลัง|ขึ้น|ลง|left|right|forward|back|up|down)", t):
            dir_map = {
                "หน้า": "forward", "forward": "forward",
                "หลัง": "back", "back": "back",
                "ซ้าย": "left", "left": "left",
                "ขวา": "right", "right": "right",
                "ขึ้น": "up", "up": "up",
                "ลง": "down", "down": "down",
            }
            return ("move_simple", dir_map.get(t, t), None)

        if re.fullmatch(r"(ซ้ายนิด|ขวานิด|ขึ้นนิด|ลงนิด)", t):
            dir_map = {
                "ซ้ายนิด": "left",
                "ขวานิด": "right",
                "ขึ้นนิด": "up",
                "ลงนิด": "down"
            }
            return ("move_simple", dir_map.get(t, None), None)

        return ("other", None, None)

    # -------------------------------------------------
    # ตรวจ direction ปกติ
    # -------------------------------------------------
    if re.search(r"(ซ้าย|left)", t):
        dir_ = "left"
    elif re.search(r"(ขวา|right)", t):
        dir_ = "right"
    elif re.search(r"(หน้า|forward)", t):
        dir_ = "forward"
    elif re.search(r"(หลัง|back|ถอย)", t):
        dir_ = "back"
    elif re.search(r"(ขึ้น|up)", t):
        dir_ = "up"
    elif re.search(r"(ลง|down)", t):
        dir_ = "down"
    else:
        return ("move_ask_dir", None, None)

    # -------------------------------------------------
    # ถ้าขอให้ระบุระยะ
    # -------------------------------------------------
    if ("ระบุระยะ" in t) or contains_fuzzy("ระบุระยะ", t) or ("ระบุจำนวน" in t):
        return ("move_ask_dist", dir_, None)

    dist = parse_distance(t)

    if dist is None:
        return ("move_simple", dir_, None)

    return ("move", dir_, dist)


def make_cmd_group(t: str) -> str:
    """Map canonical text into a compact cmd_group string for downstream mapper/executor."""
    if re.search(r"(ยกเลิก|cancel|abort)", t):
        return "CANCEL"
    if re.search(r"(หยุด|stop|พอ)", t):
        return "STOP"

    if parse_session_reset(t):
        return "SESSION_RESET"

    if re.search(r"(กลับบ้าน|กลับ\s*โฮม|กลับบ้านที|กลับไปบ้าน|back\s*home)", t):
        return "HOME"

    lu = parse_lock_unlock(t)
    if lu == "lock":
        return "LOCK"
    if lu == "unlock":
        return "UNLOCK"

    sp_kind, sp = parse_speed(t)
    if sp_kind == "speed" and sp:
        return f"SPEED_{sp.upper()}"

    v_kind, v_vk, v_pos = parse_view(t)
    if v_kind == "view" and v_vk and v_pos is not None:
        return f"TOP_VIEW_{v_pos}" if v_vk == "top" else f"SIDE_VIEW_{v_pos}"

    p_intent, p_val = parse_position(t)
    if p_intent == "pos" and p_val is not None:
        return f"POS_{p_val}"

    pp = parse_pick_place(t)
    if pp == "pick":
        return "PICK"
    if pp == "place":
        return "PLACE"

    w3_intent, w3_dir, w3_deg = parse_w3_rotate(t)
    if w3_intent in ("w3_rotate_ask_deg", "w3_rotate_ask_dir"):
        return "UNKNOWN"
    if w3_dir == "left":
        return f"W3_LEFT:{w3_deg:g}" if w3_deg is not None else "W3_LEFT"
    if w3_dir == "right":
        return f"W3_RIGHT:{w3_deg:g}" if w3_deg is not None else "W3_RIGHT"

    r_intent, r_dir, r_deg = parse_rotate(t)
    if r_intent in ("rotate_ask_deg", "rotate_ask_dir"):
        return "UNKNOWN"
    if r_dir == "left":
        return f"ROTATE_LEFT:{r_deg:g}" if r_deg is not None else "ROTATE_LEFT"
    if r_dir == "right":
        return f"ROTATE_RIGHT:{r_deg:g}" if r_deg is not None else "ROTATE_RIGHT"
    
    _, s_dir, s_deg = parse_scroll(t)
    if s_dir == "up":
        return f"SCROLL_UP:{s_deg:g}" if s_deg is not None else "SCROLL_UP"
    if s_dir == "down":
        return f"SCROLL_DOWN:{s_deg:g}" if s_deg is not None else "SCROLL_DOWN"

    _, m_dir, m_dist = parse_move(t)
    if m_dir == "left":
        return f"MOVE_LEFT:{m_dist:g}" if m_dist is not None else "MOVE_LEFT"
    if m_dir == "right":
        return f"MOVE_RIGHT:{m_dist:g}" if m_dist is not None else "MOVE_RIGHT"
    if m_dir == "forward":
        return f"MOVE_FORWARD:{m_dist:g}" if m_dist is not None else "MOVE_FORWARD"
    if m_dir == "back":
        return f"MOVE_BACK:{m_dist:g}" if m_dist is not None else "MOVE_BACK"
    if m_dir == "up":
        return f"MOVE_UP:{m_dist:g}" if m_dist is not None else "MOVE_UP"
    if m_dir == "down":
        return f"MOVE_DOWN:{m_dist:g}" if m_dist is not None else "MOVE_DOWN"

    return "UNKNOWN"


class NLUParserNode(Node):
    def __init__(self):
        super().__init__("nlu_parser_node")

        GREEN = "\033[92m"
        RESET = "\033[0m"

        self.declare_parameter("debug", True)
        self.declare_parameter("also_publish_banner_to_debug_topic", False)
        self.also_publish_banner_to_debug_topic = bool(
        self.get_parameter("also_publish_banner_to_debug_topic").value
        )

        self.debug = bool(self.get_parameter("debug").value)

        self.declare_parameter("debounce_enable", True)
        self.declare_parameter("debounce_seconds", 0.8)
        self.declare_parameter("debounce_min_chars", 3)

        self.declare_parameter("debounce_similarity_enable", True)
        self.declare_parameter("debounce_similarity_threshold", 0.92)

        self.debounce_enable = bool(self.get_parameter("debounce_enable").value)
        self.debounce_seconds = float(self.get_parameter("debounce_seconds").value)
        self.debounce_min_chars = int(self.get_parameter("debounce_min_chars").value)

        self.debounce_similarity_enable = bool(self.get_parameter("debounce_similarity_enable").value)
        self.debounce_similarity_threshold = float(self.get_parameter("debounce_similarity_threshold").value)

        self._last_canon: str = ""
        self._last_time_sec: float = 0.0

        self._locked: bool = False

        self.declare_parameter("topic_executor_state", "ur5/executor_state")
        self.topic_executor_state = str(self.get_parameter("topic_executor_state").value)
        self._executor_busy: bool = False

        self.declare_parameter("limit_rotate_cmd_deg", 120.0)
        self.declare_parameter("limit_rotate_total_deg", 360.0)
        self.declare_parameter("limit_w3_rotate_cmd_deg", 180.0)
        self.declare_parameter("limit_w3_rotate_total_deg", 720.0)

        self.limit_rotate_cmd_deg = float(self.get_parameter("limit_rotate_cmd_deg").value)
        self.limit_rotate_total_deg = float(self.get_parameter("limit_rotate_total_deg").value)
        self.limit_w3_rotate_cmd_deg = float(self.get_parameter("limit_w3_rotate_cmd_deg").value)
        self.limit_w3_rotate_total_deg = float(self.get_parameter("limit_w3_rotate_total_deg").value)

        self.declare_parameter("limit_move_cmd_m", 0.20)
        self.limit_move_cmd_m = float(self.get_parameter("limit_move_cmd_m").value)

        # Rotation accumulators (only accumulate commands with explicit degree values)
        self._acc_rotate_deg = 0.0
        self._acc_w3_rotate_deg = 0.0

        self.declare_parameter("topic_text_raw", "control/text_raw")
        self.declare_parameter("topic_intent", "Neural_parser/intent")
        self.declare_parameter("topic_cmd_group", "Neural_parser/cmd_group")
        self.declare_parameter("topic_text_canon", "Neural_parser/text_canon")
        self.declare_parameter("topic_nlu_event", "Neural_parser/nlu_event")
        self.declare_parameter("topic_nlu_debug", "Neural_parser/nlu_debug")

        self.topic_text_raw = str(self.get_parameter("topic_text_raw").value)
        self.topic_intent = str(self.get_parameter("topic_intent").value)
        self.topic_cmd_group = str(self.get_parameter("topic_cmd_group").value)
        self.topic_text_canon = str(self.get_parameter("topic_text_canon").value)
        self.topic_nlu_event = str(self.get_parameter("topic_nlu_event").value)
        self.topic_nlu_debug = str(self.get_parameter("topic_nlu_debug").value)

        self.sub = self.create_subscription(String, self.topic_text_raw, self.cb, 10)
        self.exec_state_sub = self.create_subscription(String, self.topic_executor_state, self._cb_executor_state, 10)

        self.intent_pub = self.create_publisher(String, self.topic_intent, 10)
        self.group_pub = self.create_publisher(String, self.topic_cmd_group, 10)

        self.tts_req_pub = self.create_publisher(String, "control/tts_request", 10)
        self.fb_req_pub = self.create_publisher(String, "Neural_parser/feedback_request", 10)
        self.dialog_pub = self.create_publisher(String, "Neural_parser/dialog_request", 10)

        self.canon_pub = self.create_publisher(String, self.topic_text_canon, 10)

        self.event_pub = self.create_publisher(String, self.topic_nlu_event, 10)
        self.debug_pub = self.create_publisher(String, self.topic_nlu_debug, 10)
        self.supported = [
        "STOP / CANCEL (highest priority)",
        "SESSION RESET (clear state/accumulators/unlock)",
        "HOME",
        "LOCK / UNLOCK (block other commands)",
        "BUSY/IDLE feedback (avoid command stacking)",
        "Smart debounce (drop similar sentences in short window)",
        "SPEED slow/normal/fast",
        "TOP_VIEW n / SIDE_VIEW n",
        "POS 1-5",
        "PICK / PLACE",
        "W3 ROTATE left/right + degrees (Thai number supported)",
        "ROTATE left/right + degrees (Thai number supported)",
        "ROTATE LIMITS: per-command + total (event/debug/TTS on violation)",
        "MOVE directions + distance",
        "MOVE LIMIT: per-command not exceeding 0.20 m (param limit_move_cmd_m)",
        "Dialog asks: ASK_POS / ASK_MOVE_DIST / ASK_ROTATE_DEG / ASK_W3_ROTATE_DEG / ASK_VIEW_POS",
        ]
        self._print_startup_banner()
        self._pending_view: Optional[str] = None
        self._dialog_state: Optional[str] = None

    def _print_startup_banner(self):
        GREEN = "\033[92m"
        RESET = "\033[0m"

        def _safe(v, default="(n/a)"):
            try:
                return v
            except Exception:
                return default

        supported = self.supported if hasattr(self, "supported") else []

        banner = (
            "\n"
            "──────────────────────────────────────────────────────────────\n"
            "        NLU Parser Node — Operational\n"
            "        Node State      : READY\n"
            f"        Debug Mode      : {'ENABLED' if self.debug else 'DISABLED'}\n"
            f"        Debug Topic     : {_safe(self.topic_nlu_debug) if self.debug else '(disabled)'}\n"
            f"        Event Topic     : {_safe(self.topic_nlu_event)}\n"
            "\n"
            "        Subscribed Topics:\n"
            f"            • {_safe(self.topic_text_raw)}\n"
            f"            • {_safe(self.topic_executor_state)}   (BUSY/IDLE)\n"
            "\n"
            "        Published Topics:\n"
            f"            • {_safe(self.topic_text_canon)}\n"
            f"            • {_safe(self.topic_intent)}\n"
            f"            • {_safe(self.topic_cmd_group)}\n"
            f"            • {_safe(self.topic_nlu_event)}\n"
            f"            • {_safe(self.topic_nlu_debug) if self.debug else '(disabled)'}\n"
            "            • voice/tts_request\n"
            "            • voice/feedback_request\n"
            "            • voice/dialog_request\n"
            "\n"
            "        Runtime Config:\n"
            f"            • debounce_enable   = {self.debounce_enable}\n"
            f"            • debounce_seconds  = {self.debounce_seconds}\n"
            f"            • debounce_min_chars= {self.debounce_min_chars}\n"
            f"            • sim_debounce      = {self.debounce_similarity_enable} (th={self.debounce_similarity_threshold})\n"
            f"            • limit_move_cmd_m  = {self.limit_move_cmd_m}\n"
            f"            • rotate_cmd_limit  = {self.limit_rotate_cmd_deg} deg\n"
            f"            • rotate_total_lim  = {self.limit_rotate_total_deg} deg\n"
            f"            • w3_cmd_limit      = {self.limit_w3_rotate_cmd_deg} deg\n"
            f"            • w3_total_limit    = {self.limit_w3_rotate_total_deg} deg\n"
            f"            • initial_locked    = {self._locked}\n"
            f"            • initial_exec_busy = {self._executor_busy}\n"
            "\n"
            "        Supported Functions:\n"
            "            - " + "\n            - ".join(supported) + "\n"
            "──────────────────────────────────────────────────────────────\n"
        )
        self.get_logger().info(GREEN + banner + RESET)
        if self.also_publish_banner_to_debug_topic and self.debug:
            try:
                self.debug_pub.publish(String(data=f"[DEBUG][nlu] banner:\n{banner}"))
            except Exception:
                pass

    def _dbg(self, msg: str) -> None:
        """Publish debug message to the debug topic if debug mode is enabled."""
        if self.debug:
            self.debug_pub.publish(String(data=f"[DEBUG][nlu] {msg}"))

    def _event(self, msg: str) -> None:
        """Publish an NLU event message (for logging/monitoring)."""
        self.event_pub.publish(String(data=msg))

    def _tts(self, text: str) -> None:
        """Request TTS output."""
        self.tts_req_pub.publish(String(data=text))

    def _fb(self, text: str) -> None:
        """Request user feedback (UI/console layer)."""
        self.fb_req_pub.publish(String(data=text))

    def _dialog(self, text: str) -> None:
        """Request a dialog action (ask user for missing slot/value)."""
        self.dialog_pub.publish(String(data=text))

    def _pub_intent(self, intent: str) -> None:
        """Publish a normalized intent string."""
        self.intent_pub.publish(String(data=intent))
        self._dbg(f"INTENT -> {intent}")

    def _pub_group(self, group_cmd: str) -> None:
        """Publish a compact command group token for downstream mapper/executor."""
        self.group_pub.publish(String(data=group_cmd))
        self._dbg(f"GROUP -> {group_cmd}")

    def _now_sec(self) -> float:
        """Return current node clock time in seconds (float)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _similar(self, a: str, b: str) -> float:
        """Compute similarity between two strings (SequenceMatcher ratio)."""
        return difflib.SequenceMatcher(None, a or "", b or "").ratio()

    def _should_drop_by_debounce(self, canon: str) -> bool:
        """Decide whether to drop current input due to debounce or similarity debounce."""
        if not self.debounce_enable:
            return False

        c = (canon or "").strip()
        if len(c) < self.debounce_min_chars:
            return False

        now = self._now_sec()

        if c == self._last_canon and (now - self._last_time_sec) <= self.debounce_seconds:
            return True

        if self.debounce_similarity_enable and self._last_canon:
            if (now - self._last_time_sec) <= self.debounce_seconds:
                sim = self._similar(c, self._last_canon)
                if sim >= self.debounce_similarity_threshold:
                    self._dbg(f"drop: similarity debounce sim={sim:.2f} last='{self._last_canon}' now='{c}'")
                    return True

        self._last_canon = c
        self._last_time_sec = now
        return False

    def _is_allowed_when_locked(self, canon: str) -> bool:
        """Return True if this command is allowed to pass while locked."""
        t = canon.lower()
        if re.search(r"(ยกเลิก|cancel|abort)", t):
            return True
        if re.search(r"(หยุด|stop|พอ)", t):
            return True
        if parse_session_reset(t):
            return True
        lu = parse_lock_unlock(t)
        if lu == "unlock":
            return True
        return False

    def _is_allowed_when_busy(self, canon: str) -> bool:
        """Return True if this command is allowed to pass while executor is BUSY."""
        t = canon.lower()
        if re.search(r"(ยกเลิก|cancel|abort)", t):
            return True
        if re.search(r"(หยุด|stop|พอ)", t):
            return True
        if parse_session_reset(t):
            return True
        return False

    def _cb_executor_state(self, msg: String) -> None:
        """Update internal busy flag from executor state messages (BUSY/IDLE)."""
        s = (msg.data or "").strip().upper()
        if "BUSY" in s:
            if not self._executor_busy:
                self._dbg("executor_state -> BUSY")
            self._executor_busy = True
        elif "IDLE" in s:
            if self._executor_busy:
                self._dbg("executor_state -> IDLE")
            self._executor_busy = False
        else:
            self._dbg(f"executor_state unknown '{s}' (expect BUSY/IDLE)")

    def _abs_over(self, v: Optional[float], limit: float) -> bool:
        """Check if absolute value exceeds the given limit."""
        if v is None:
            return False
        return abs(float(v)) > float(limit)

    def _would_exceed_total(self, current_total: float, delta_deg: float, total_limit: float) -> bool:
        """Check if adding delta would exceed total accumulation limit."""
        return (abs(float(current_total)) + abs(float(delta_deg))) > abs(float(total_limit))

    def _reset_rotate_acc(self, reason: str = "") -> None:
        """Reset rotate accumulators for rotate and wrist3 rotate."""
        self._acc_rotate_deg = 0.0
        self._acc_w3_rotate_deg = 0.0
        if reason:
            self._dbg(f"reset rotate accumulators: {reason}")

    def _reset_session(self, reason: str = "SESSION_RESET") -> None:
        """Reset session state: clear debounce history, unlock, and clear accumulators."""
        self._last_canon = ""
        self._last_time_sec = 0.0
        self._locked = False
        self._reset_rotate_acc(reason)
        self._dbg(f"session reset done: {reason}")

    def _block_rotate_limit(self, *, kind: str, which: str, value: float, limit: float, total_before: float) -> None:
        """Block rotate command when command-limit or total-limit is exceeded and publish safety feedback."""
        self._pub_group("UNKNOWN")

        if which == "CMD":
            intent = f"{kind}:OVER_CMD_LIMIT"
            event = f"SAFETY:{kind}:CMD_LIMIT:value={value:g}:limit={limit:g}"
            dbg = f"SAFETY:{kind}:CMD_LIMIT value={value:g} limit={limit:g}"
            fb = f"เกินขีดจำกัด: {('ข้อมือสาม' if kind=='W3_ROTATE' else 'หมุน')} ได้ไม่เกิน ±{limit:g} องศาต่อครั้ง"
            tts = f"เกินลิมิต หมุนได้ไม่เกิน {limit:g} องศา"
        else:
            intent = f"{kind}:OVER_TOTAL_LIMIT"
            event = f"SAFETY:{kind}:TOTAL_LIMIT:add={value:g}:total={total_before:g}:limit={limit:g}"
            dbg = f"SAFETY:{kind}:TOTAL_LIMIT add={value:g} total={total_before:g} limit={limit:g}"
            fb = f"เกินเพดานหมุนสะสม: limit {limit:g} องศา (ตอนนี้สะสม {total_before:g})"
            tts = "เกินเพดานการหมุนสะสม"

        self._pub_intent(intent)
        self._event(event)
        self._dbg(dbg)
        self._fb(fb)
        self._tts(tts)

    def _block_move_limit(self, *, value_m: float, limit_m: float) -> None:
        """Block move command when per-command distance limit is exceeded and publish safety feedback."""
        self._pub_group("UNKNOWN")
        intent = "MOVE:OVER_CMD_LIMIT"
        event = f"SAFETY:MOVE:CMD_LIMIT:value={value_m:g}:limit={limit_m:g}"
        dbg = f"SAFETY:MOVE:CMD_LIMIT value={value_m:g} limit={limit_m:g}"
        fb = f"เกินขีดจำกัด: MOVE ต่อครั้งได้ไม่เกิน {limit_m:g} เมตร (คุณสั่ง {value_m:g})"
        tts = f"เกินลิมิต ระยะต้องไม่เกิน {limit_m:g} เมตร"
        self._pub_intent(intent)
        self._event(event)
        self._dbg(dbg)
        self._fb(fb)
        self._tts(tts)

    def cb(self, msg: String) -> None:
        """Main callback: normalize text, apply debounce/locks/busy gates, then parse and publish intents/groups."""
        raw_in = normalize_thai(msg.data)
        if not raw_in:
            self._dbg("drop: empty raw")
            return

        t = normalize_for_nlu(raw_in)
        self.canon_pub.publish(String(data=t))

        if self._should_drop_by_debounce(t):
            self._dbg(f"drop: debounce canon='{t}'")
            return

        if re.search(r"(ยกเลิก|cancel|abort)", t):
            self._pub_group("CANCEL")
            self._pub_intent("CANCEL")
            self._event("ACK:CANCEL")
            self._fb("รับคำสั่งแล้ว: ยกเลิกการเคลื่อนที่")
            self._tts("ยกเลิกการเคลื่อนที่")
            return

        if re.search(r"(หยุด|stop|พอ)", t):
            self._pub_group("STOP")
            self._pub_intent("STOP")
            self._event("ACK:STOP")
            self._fb("รับคำสั่งแล้ว: หยุด")
            self._tts("หยุด")
            return

        if parse_session_reset(t):
            self._reset_session("SESSION_RESET")
            self._pub_group("SESSION_RESET")
            self._pub_intent("SESSION_RESET")
            self._event("ACK:SESSION_RESET")
            self._fb("รีเซ็ตเซสชันแล้ว: ปลดล็อก + ล้างค่าสะสม + ล้างดีบาวซ์")
            self._tts("รีเซ็ตแล้ว")
            return

        if self._locked and not self._is_allowed_when_locked(t):
            self._pub_group("UNKNOWN")
            self._pub_intent("LOCKED:BLOCK")
            self._event("BLOCK:LOCKED")
            self._fb("ระบบล็อกอยู่ครับ ต้องปลดล็อกก่อน")
            self._tts("ระบบล็อกอยู่ ต้องปลดล็อกก่อน")
            return

        if self._executor_busy and not self._is_allowed_when_busy(t):
            self._pub_group("UNKNOWN")
            self._pub_intent("BUSY:BLOCK")
            self._event("BLOCK:BUSY")
            self._fb("ตอนนี้กำลังเคลื่อนที่อยู่ครับ ถ้าต้องการหยุดให้พูดว่า หยุด หรือ ยกเลิก")
            self._tts("กำลังเคลื่อนที่อยู่ ถ้าต้องการหยุดให้พูดว่า หยุด หรือ ยกเลิก")
            return

        if re.search(r"(กลับบ้าน|กลับ\s*โฮม|กลับบ้านที|กลับไปบ้าน|back\s*home)", t):
            self._reset_rotate_acc("HOME")
            self._pub_group("HOME")
            self._pub_intent("HOME")
            self._event("ACK:HOME")
            self._fb("รับคำสั่งแล้ว: กลับ Home")
            self._tts("Back to Home")
            return

        lu = parse_lock_unlock(t)
        if lu == "lock":
            self._locked = True
            self._pub_group("LOCK")
            self._pub_intent("LOCK")
            self._event("ACK:LOCK")
            self._fb("รับคำสั่งแล้ว: ล็อกระบบ")
            self._tts("ล็อกระบบแล้ว")
            return
        if lu == "unlock":
            self._locked = False
            self._pub_group("UNLOCK")
            self._pub_intent("UNLOCK")
            self._event("ACK:UNLOCK")
            self._fb("รับคำสั่งแล้ว: ปลดล็อกระบบ")
            self._tts("ปลดล็อกระบบแล้ว")
            return

        sp_kind, sp = parse_speed(t)
        if sp_kind == "speed" and sp:
            grp = f"SPEED_{sp.upper()}"
            self._pub_group(grp)
            self._pub_intent(f"SPEED:{sp}")
            self._event(f"ACK:{grp}")
            th = {"slow": "ช้า", "normal": "ปกติ", "fast": "เร็ว"}.get(sp, sp)
            self._fb(f"รับคำสั่งแล้ว: ความเร็ว{th}")
            self._tts(f"ปรับความเร็วเป็น{th}") 
            return
        
        pp = parse_pick_place(t)
        if pp == "pick":
            self._pending_view = None
            self._dialog_state = None
            self._pub_group("PICK")
            self._pub_intent("PICK")
            self._event("ACK:PICK")
            self._fb("รับคำสั่งแล้ว: หยิบของ")
            self._tts("กำลังดำเนินการ หยิบของ")
            return

        if pp == "place":
            self._pending_view = None
            self._dialog_state = None
            self._pub_group("PLACE")
            self._pub_intent("PLACE")
            self._event("ACK:PLACE")
            self._fb("รับคำสั่งแล้ว: วางของ")
            self._tts("กำลังดำเนินการ วางของ")
            return
        
        if self._dialog_state == "ASK_VIEW_POS" and self._pending_view:
            vk = self._pending_view
            self._pending_view = None
            self._dialog_state = None
            p_intent, p_val = parse_position(t)

            if p_intent == "pos" and p_val is not None:
                grp = f"TOP_VIEW_{p_val}" if vk == "top" else f"SIDE_VIEW_{p_val}"
                self._pub_group(grp)
                self._pub_intent(f"{vk.upper()}_VIEW:{p_val}")
                self._event(f"ACK:{grp}")
                self._fb(f"รับคำสั่งแล้ว: มุมมอง{'บน' if vk=='top' else 'ข้าง'} ตำแหน่งที่ {p_val}")
                self._tts(f"มุมมอง{'บน' if vk=='top' else 'ข้าง'} ตำแหน่งที่ {p_val}")
                return
            
            else:
                self._pending_view = None

        v_kind, v_vk, v_pos = parse_view(t)
        if v_kind == "view" and v_vk and (v_pos is not None):
            if v_vk == "top":
                grp = f"TOP_VIEW_{v_pos}"
                self._pub_group(grp)
                self._pub_intent(f"TOP_VIEW:{v_pos}")
                self._event(f"ACK:{grp}")
                self._fb(f"รับคำสั่งแล้ว: มุมมองบน ตำแหน่งที่ {v_pos}")
                self._tts(f"มุมมองบน ตำแหน่งที่ {v_pos}")
                return

            if v_vk == "side":
                grp = f"SIDE_VIEW_{v_pos}"
                self._pub_group(grp)
                self._pub_intent(f"SIDE_VIEW:{v_pos}")
                self._event(f"ACK:{grp}")
                self._fb(f"รับคำสั่งแล้ว: มุมมองข้าง ตำแหน่งที่ {v_pos}")
                self._tts(f"มุมมองข้าง ตำแหน่งที่ {v_pos}")
                return

        if v_kind == "view_ask_pos":
            self._pub_group("UNKNOWN")
            self._pending_view = v_vk
            self._dialog_state = "ASK_VIEW_POS"
            if v_vk in ("top", "side"):
                self._dialog(f"ASK_VIEW_POS:{v_vk}")
                self._pub_intent(f"VIEW:{v_vk}:ASK_POS")
                self._event(f"DIALOG:ASK_VIEW_POS:{v_vk}")
                self._fb("ต้องการมุมมองที่ตำแหน่งไหน 1 2 3 4 หรือ 5?")
                self._tts("ต้องการตำแหน่งไหน 1 2 3 4 หรือ 5")
            else:
                self._dialog("ASK_VIEW_POS")
                self._pub_intent("VIEW:ASK_POS")
                self._event("DIALOG:ASK_VIEW_POS")
                self._fb("ต้องการมุมมองที่ตำแหน่งไหน 1 2 3 4 หรือ 5?")
                self._tts("ต้องการตำแหน่งไหน 1 2 3 4 หรือ 5")
            return
        
        p_intent, p_val = parse_position(t)
        if p_intent == "pos" and p_val is not None:
            self._reset_rotate_acc(f"POS_{p_val}")
            grp = f"POS_{p_val}"
            self._pub_group(grp)
            self._pub_intent(f"POS:{p_val}")
            self._event(f"ACK:{grp}")
            self._fb(f"รับคำสั่งแล้ว: ตำแหน่งที่ {p_val}")
            self._tts(f"กำลังดำเนินการ ตำแหน่งที่ {p_val}")
            return

        if p_intent == "pos_ask":
            self._pub_group("UNKNOWN")
            self._dialog("ASK_POS")
            self._pub_intent("POS:ASK")
            self._event("DIALOG:ASK_POS")
            self._fb("คุณต้องการตำแหน่งที่ 1 2 3 4 หรือ 5?")
            self._tts("คุณหมายถึงตำแหน่งไหน เลือก 1 2 3 4 หรือ 5")
            return

        pp = parse_pick_place(t)
        if pp == "pick":
            self._pub_group("PICK")
            self._pub_intent("PICK")
            self._event("ACK:PICK")
            self._fb("รับคำสั่งแล้ว: หยิบของ")
            self._tts("กำลังดำเนินการ หยิบของ")
            return

        if pp == "place":
            self._pub_group("PLACE")
            self._pub_intent("PLACE")
            self._event("ACK:PLACE")
            self._fb("รับคำสั่งแล้ว: วางของ")
            self._tts("กำลังดำเนินการ วางของ")
            return

        w3_intent, w3_dir, w3_deg = parse_w3_rotate(t)

        if w3_intent == "w3_rotate_ask_dir":
            self._pub_group("UNKNOWN")
            self._dialog("ASK_W3_DIR")
            self._pub_intent("ROTATE_W3:ASK_DIR")
            self._event("DIALOG:ASK_W3_DIR")
            self._fb("ต้องการหมุนข้อมือสามซ้ายหรือขวา?")
            self._tts("ต้องการหมุนข้อมือสามซ้ายหรือขวา")
            return

        if w3_intent == "w3_rotate_ask_deg" and w3_dir:
            self._pub_group("UNKNOWN")
            self._dialog(f"ASK_W3_ROTATE_DEG:{w3_dir}")
            self._pub_intent(f"ROTATE_W3:{w3_dir}:ASK_DEG")
            self._event(f"DIALOG:ASK_W3_ROTATE_DEG:{w3_dir}")
            self._fb(f"ต้องการหมุนข้อมือสาม{'ซ้าย' if w3_dir=='left' else 'ขวา'}กี่องศา?")
            self._tts("ต้องการหมุนข้อมือสามกี่องศา เช่น 5 หรือ 20")
            return

        if w3_intent == "w3_rotate" and w3_dir:
            if w3_deg is not None:
                if self._abs_over(w3_deg, self.limit_w3_rotate_cmd_deg):
                    self._block_rotate_limit(
                        kind="W3_ROTATE",
                        which="CMD",
                        value=float(w3_deg),
                        limit=self.limit_w3_rotate_cmd_deg,
                        total_before=self._acc_w3_rotate_deg,
                    )
                    return

                if self._would_exceed_total(self._acc_w3_rotate_deg, float(w3_deg), self.limit_w3_rotate_total_deg):
                    self._block_rotate_limit(
                        kind="W3_ROTATE",
                        which="TOTAL",
                        value=float(w3_deg),
                        limit=self.limit_w3_rotate_total_deg,
                        total_before=self._acc_w3_rotate_deg,
                    )
                    return

            grp = f"W3_LEFT:{w3_deg:g}" if (w3_dir == "left" and w3_deg is not None) else \
                  "W3_LEFT" if (w3_dir == "left") else \
                  f"W3_RIGHT:{w3_deg:g}" if (w3_deg is not None) else "W3_RIGHT"

            self._pub_group(grp)

            if w3_deg is not None:
                self._acc_w3_rotate_deg += abs(float(w3_deg))
                self._dbg(f"acc_w3_rotate_deg -> {self._acc_w3_rotate_deg:g} (add {abs(float(w3_deg)):g})")

            if w3_deg is None:
                self._pub_intent(f"ROTATE_W3:{w3_dir}:DEFAULT")
                self._event(f"ACK:{grp}")
                self._fb("รับคำสั่งแล้ว: หมุนข้อมือสาม")
                self._tts("กำลังดำเนินการ หมุนข้อมือสาม")
            else:
                self._pub_intent(f"ROTATE_W3:{w3_dir}:{w3_deg:g}")
                self._event(f"ACK:{grp}")
                self._fb(f"รับคำสั่งแล้ว: หมุนข้อมือสาม{'ซ้าย' if w3_dir=='left' else 'ขวา'} {w3_deg:g} องศา")
                self._tts(f"กำลังดำเนินการ หมุนข้อมือสาม{'ซ้าย' if w3_dir=='left' else 'ขวา'} {w3_deg:g} องศา")
            return

        r_intent, r_dir, r_deg = parse_rotate(t)

        if r_intent == "rotate_ask_dir":
            self._pub_group("UNKNOWN")
            self._dialog("ASK_ROTATE_DIR")
            self._pub_intent("ROTATE:ASK_DIR")
            self._event("DIALOG:ASK_ROTATE_DIR")
            self._fb("ต้องการหมุนซ้ายหรือขวา?")
            self._tts("ต้องการหมุนซ้ายหรือขวา")
            return

        if r_intent == "rotate_ask_deg" and r_dir:
            self._pub_group("UNKNOWN")
            self._dialog(f"ASK_ROTATE_DEG:{r_dir}")
            self._pub_intent(f"ROTATE:{r_dir}:ASK_DEG")
            self._event(f"DIALOG:ASK_ROTATE_DEG:{r_dir}")
            self._fb(f"ต้องการหมุน{'ซ้าย' if r_dir=='left' else 'ขวา'}กี่องศา?")
            self._tts("ต้องการหมุนกี่องศา เช่น 5 หรือ 20")
            return

        if r_intent == "rotate" and r_dir:
            if r_deg is not None:
                if self._abs_over(r_deg, self.limit_rotate_cmd_deg):
                    self._block_rotate_limit(
                        kind="ROTATE",
                        which="CMD",
                        value=float(r_deg),
                        limit=self.limit_rotate_cmd_deg,
                        total_before=self._acc_rotate_deg,
                    )
                    return

                if self._would_exceed_total(self._acc_rotate_deg, float(r_deg), self.limit_rotate_total_deg):
                    self._block_rotate_limit(
                        kind="ROTATE",
                        which="TOTAL",
                        value=float(r_deg),
                        limit=self.limit_rotate_total_deg,
                        total_before=self._acc_rotate_deg,
                    )
                    return

            grp = f"ROTATE_LEFT:{r_deg:g}" if (r_dir == "left" and r_deg is not None) else \
                  "ROTATE_LEFT" if (r_dir == "left") else \
                  f"ROTATE_RIGHT:{r_deg:g}" if (r_deg is not None) else "ROTATE_RIGHT"

            self._pub_group(grp)

            if r_deg is not None:
                self._acc_rotate_deg += abs(float(r_deg))
                self._dbg(f"acc_rotate_deg -> {self._acc_rotate_deg:g} (add {abs(float(r_deg)):g})")

            if r_deg is None:
                self._pub_intent(f"ROTATE:{r_dir}:DEFAULT")
                self._event(f"ACK:{grp}")
                self._fb("รับคำสั่งแล้ว: หมุน")
                self._tts("กำลังดำเนินการ หมุน")
            else:
                self._pub_intent(f"ROTATE:{r_dir}:{r_deg:g}")
                self._event(f"ACK:{grp}")
                self._fb(f"รับคำสั่งแล้ว: หมุน{'ซ้าย' if r_dir=='left' else 'ขวา'} {r_deg:g} องศา")
                self._tts(f"กำลังดำเนินการ หมุน{'ซ้าย' if r_dir=='left' else 'ขวา'} {r_deg:g} องศา")
            return
        
        s_intent, s_dir, s_deg = parse_scroll(t)
        if s_intent == "scroll_ask_dir":
            self._pub_group("UNKNOWN")
            self._dialog("ASK_SCROLL_DIR")
            self._pub_intent("SCROLL:ASK_DIR")
            self._event("DIALOG:ASK_SCROLL_DIR")
            self._fb("คุณต้องการเลื่อนขึ้นหรือเลื่อนลง?")
            self._tts("ต้องการเลื่อนขึ้นหรือเลื่อนลง")
            return

        if s_intent == "scroll_ask_deg" and s_dir:
            self._pub_group("UNKNOWN")
            self._dialog(f"ASK_SCROLL_DEG:{s_dir}")
            self._pub_intent(f"SCROLL:{s_dir}:ASK_DEG")
            self._event(f"DIALOG:ASK_SCROLL_DEG:{s_dir}")
            self._fb("ต้องการกี่องศา เช่น 15 หรือ 20")
            self._tts("ต้องการกี่องศา เช่น 15 หรือ 20")
            return

        if s_intent in ("scroll_simple", "scroll") and s_dir:
            self._pub_group("UNKNOWN")
            if s_deg is None:
                self._pub_intent(f"SCROLL:{s_dir}:DEFAULT")
                self._event(f"ACK:SCROLL:{s_dir}:DEFAULT")
                self._fb("รับคำสั่งแล้ว: เลื่อน")
                self._tts("กำลังดำเนินการ เลื่อน")
            else:
                self._pub_intent(f"SCROLL:{s_dir}:{s_deg:g}")
                self._event(f"ACK:SCROLL:{s_dir}:{s_deg:g}")
                self._fb(f"รับคำสั่งแล้ว: เลื่อน{'ขึ้น' if s_dir=='up' else 'ลง'} {s_deg:g} องศา")
                self._tts(f"กำลังดำเนินการ เลื่อน{'ขึ้น' if s_dir=='up' else 'ลง'} {s_deg:g} องศา")
            return

        m_intent, m_dir, m_dist = parse_move(t)

        if m_intent == "move_ask_dir":
            self._pub_group("UNKNOWN")
            self._dialog("ASK_MOVE_DIR")
            self._pub_intent("MOVE:ASK_DIR")
            self._event("DIALOG:ASK_MOVE_DIR")
            self._fb("ต้องการขยับไปทางไหน ซ้าย ขวา หน้า หลัง ขึ้น ลง?")
            self._tts("ต้องการขยับไปทางไหน ซ้าย ขวา หน้า หลัง ขึ้น ลง")
            return

        if m_intent == "move_ask_dist" and m_dir:
            self._pub_group("UNKNOWN")
            self._dialog(f"ASK_MOVE_DIST:{m_dir}")
            self._pub_intent(f"MOVE:{m_dir}:ASK_DIST")
            self._event(f"DIALOG:ASK_MOVE_DIST:{m_dir}")
            self._fb("ต้องการระยะเท่าไร เช่น 10 เซน หรือ 0.1 เมตร")
            self._tts("ต้องการระยะเท่าไร เช่น 10 เซน หรือ 0.1 เมตร")
            return

        if m_intent == "move_simple" and m_dir:
            dir2grp = {
                "left": "MOVE_LEFT",
                "right": "MOVE_RIGHT",
                "forward": "MOVE_FORWARD",
                "back": "MOVE_BACK",
                "up": "MOVE_UP",
                "down": "MOVE_DOWN",
            }
            grp = dir2grp.get(m_dir, "UNKNOWN")
            self._pub_group(grp)
            self._pub_intent(f"MOVE:{m_dir}:DEFAULT")
            self._event(f"ACK:{grp}")
            th_dir = {"left": "ซ้าย", "right": "ขวา", "forward": "หน้า", "back": "หลัง", "up": "ขึ้น", "down": "ลง"}.get(m_dir, m_dir)
            self._fb(f"รับคำสั่งแล้ว: ขยับ{th_dir}")
            self._tts("กำลังดำเนินการ")
            return

        if m_intent == "move" and m_dir and (m_dist is not None):
            if abs(float(m_dist)) > float(self.limit_move_cmd_m):
                self._block_move_limit(value_m=float(m_dist), limit_m=float(self.limit_move_cmd_m))
                return
            
            dir2grp = {
                "left": f"MOVE_LEFT:{m_dist:g}",
                "right": f"MOVE_RIGHT:{m_dist:g}",
                "forward": f"MOVE_FORWARD:{m_dist:g}",
                "back": f"MOVE_BACK:{m_dist:g}",
                "up": f"MOVE_UP:{m_dist:g}",
                "down": f"MOVE_DOWN:{m_dist:g}",
            }
            grp = dir2grp.get(m_dir, "UNKNOWN")
            self._pub_group(grp)
            self._pub_intent(f"MOVE:{m_dir}:{m_dist:g}")
            self._event(f"ACK:{grp}")
            th_dir = {"left": "ซ้าย", "right": "ขวา", "forward": "หน้า", "back": "หลัง", "up": "ขึ้น", "down": "ลง"}.get(m_dir, m_dir)
            self._fb(f"รับคำสั่งแล้ว: ขยับ{th_dir} {m_dist:g} เมตร")
            self._tts("กำลังดำเนินการ")
            return

        self._pub_group("UNKNOWN")
        self._pub_intent("UNKNOWN")
        self._event(f"ERR:FALLBACK:canon='{t}'")
        self._fb(f"ไม่มั่นใจคำสั่ง: {t}")
        self._tts("ขอโทษค่ะ พูดใหม่อีกครั้งได้ไหม")

def main(args=None):
    rclpy.init(args=args)
    node = NLUParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
