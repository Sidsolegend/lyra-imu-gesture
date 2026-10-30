// =============================================================================
// LYRA glasses - script parsing + word wrap (plain C++, no Arduino types, so
// tests/test_glasses_cues.py can compile it on the host)
// =============================================================================
// Script format, one line at a time:
//   ## Title        starts a new slide (the title is not shown). A "##" before any
//                   cue of the current slide does not add a slide, so a script that
//                   opens with "## Intro" starts at slide 1.
//   text lines      one cue; a blank line ends it. Words wrap to `cols`; a word
//                   longer than a line is hard-split; text past `maxLines` is cut
//                   and reported through onCut.
// =============================================================================
#pragma once
#include <stdint.h>
#include <string.h>

namespace cues {

static constexpr int MAX_COLS = 21, MAX_LINES = 4, MAX_TEXT = 512;

struct Cue { uint8_t slide; uint8_t lines; char text[MAX_LINES][MAX_COLS + 1]; };

// Word-wrap s into c, cols wide (<= MAX_COLS), at most maxLines (<= MAX_LINES).
// Returns false if the text did not fit (c holds the lines that did).
inline bool wrap(const char* s, int cols, int maxLines, Cue* c) {
  c->lines = 0; memset(c->text, 0, sizeof(c->text));
  int n = (int)strlen(s), i = 0;
  while (i < n) {
    while (i < n && s[i] == ' ') i++;
    if (i >= n) break;
    if (c->lines >= maxLines) return false;
    int end = i + cols < n ? i + cols : n;
    if (end < n && s[end] != ' ') {                // back up to a word boundary
      int sp = end;
      while (sp > i && s[sp] != ' ') sp--;
      if (sp > i) end = sp;
    }
    int e = end;
    while (e > i && s[e - 1] == ' ') e--;
    memcpy(c->text[c->lines++], s + i, e - i);
    i = end;
  }
  return true;
}

struct Parser {
  Cue* cues = nullptr;
  int maxCues = 0, cols = 12, maxLines = 3;
  int count = 0, slide = 1, lineNo = 0;
  bool dropped = false;                            // ran out of cue slots
  void (*onCut)(int cueNo, int lineNo) = nullptr;  // cueNo is 1-based
  char text[MAX_TEXT];
  int len = 0;

  void begin(Cue* c, int maxC, int cols_, int maxLines_) {
    cues = c; maxCues = maxC; cols = cols_; maxLines = maxLines_;
    count = 0; slide = 1; lineNo = 0; dropped = false; len = 0; text[0] = 0;
  }

  void flush() {
    if (!len) return;
    if (count >= maxCues) { dropped = true; len = 0; return; }
    Cue* c = &cues[count];
    c->slide = (uint8_t)slide;
    bool fit = wrap(text, cols, maxLines, c);
    count++;
    if (!fit && onCut) onCut(count, lineNo);
    len = 0; text[0] = 0;
  }

  void line(const char* raw) {
    lineNo++;
    const char* a = raw;
    while (*a == ' ' || *a == '\t') a++;
    int n = (int)strlen(a);
    while (n > 0 && (a[n - 1] == ' ' || a[n - 1] == '\t' || a[n - 1] == '\r' || a[n - 1] == '\n')) n--;
    if (n >= 2 && a[0] == '#' && a[1] == '#') {
      flush();
      if (count > 0 && cues[count - 1].slide == slide) slide++;
      return;
    }
    if (!n) { flush(); return; }
    if (len && len < MAX_TEXT - 1) text[len++] = ' ';
    int room = MAX_TEXT - 1 - len;
    if (n > room) n = room;                        // over-long cue: wrap() reports the cut
    memcpy(text + len, a, n); len += n; text[len] = 0;
  }

  void finish() { flush(); }
  int slides() const { return count ? cues[count - 1].slide : 1; }
};

}  // namespace cues
