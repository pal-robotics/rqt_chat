^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_chat
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2025-02-28)
------------------
* [minor] linting
* rewrote rqt_chat to use more advanced widgets + fix the intent display
* Contributors: Séverin Lemaignan

1.3.0 (2024-12-11)
------------------
* code refactoring + hide/show all past intents when ticking the checkbox
* Contributors: Séverin Lemaignan

1.2.0 (2024-12-10)
------------------
* clean up the 'PROCESSING_MSG' logic
* add license headers
* processing item as constant
* avoid overwriting said text
* Contributors: Sara Cooper, Séverin Lemaignan

1.1.0 (2024-12-09)
------------------
* optionally display incoming intents in the chat window
* Merge branch 'add-processing-feedback' into 'main'
  Add processing feedback
  See merge request interaction/rqt_chat!1
* revert back to using communication hub say
* publish expresion if avialable
* add timeout between id pub and speech pub
* add waiting icon
* Contributors: Sara Cooper, Séverin Lemaignan

1.0.1 (2024-11-05)
------------------
* fix linting issues
* add Apache 2 LICENSE + CONTRIBUTING.md
* fix race issue + publish word feedback
* Contributors: Séverin Lemaignan

1.0.0 (2024-10-08)
------------------
* return a TTS Result in the /say action callback
* add minimal readme
* refine UI
* initial chat plugin implementation
  - send msg to anonymous_speaker/speech and expose a /tts_engine/tts action server
* Contributors: Séverin Lemaignan
