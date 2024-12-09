^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_chat
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
