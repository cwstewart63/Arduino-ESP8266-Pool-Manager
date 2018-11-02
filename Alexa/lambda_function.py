"""
This skill returns data from a thinkspeak channel.  In my case, this is to get the pool temperature and other status information.

"Alexa ask my pool what's the pool temperature" to get the latest temperature reading.

You can also change the pool mode to either auto, on or off.

August, 2018
cwstewart63@yahoo.com

"""

from __future__ import print_function
import urllib2
import json

# Change these thingspeak elements to point to your data
# Setup two thinspeak channels, one for status (read) information, one for command (write) information
readchannel = 123456
writechannel = 654321
readapi_key = "1234ABCD5678EFGH"
writeapi_key = "ABCD1234EFGH5678"
temp_field = "field1"
mode_field = "field2"
spa_field = "field3"
pump_field = "field4"
cmd_field = "field1"
#

link = "https://api.thingspeak.com/channels/" + \
        str(readchannel) + \
        "/feeds/last.json?api_key=" + readapi_key

linkcmd0 = "https://api.thingspeak.com/update?api_key=" + \
        writeapi_key + "&" + cmd_field + "=0"
  
linkcmd1 = "https://api.thingspeak.com/update?api_key=" + \
        writeapi_key + "&" + cmd_field + "=1"
		
linkcmd2 = "https://api.thingspeak.com/update?api_key=" + \
        writeapi_key + "&" + cmd_field + "=2"

def lambda_handler(event, context):
    """ Route the incoming request based on type (LaunchRequest, IntentRequest,
    etc.) The JSON body of the request is provided in the event parameter.
    """
    print("event.session.application.applicationId=" +
          event['session']['application']['applicationId'])

    """
    Uncomment this if statement and populate with your skill's application ID to
    prevent someone else from configuring a skill that sends requests to this
    function.
    """
    # if (event['session']['application']['applicationId'] !=
    #         "amzn1.echo-sdk-ams.app.[unique-value-here]"):
    #     raise ValueError("Invalid Application ID")

    if event['session']['new']:
        on_session_started({'requestId': event['request']['requestId']},
                           event['session'])

    if event['request']['type'] == "LaunchRequest":
        return on_launch(event['request'], event['session'])
    elif event['request']['type'] == "IntentRequest":
        return on_intent(event['request'], event['session'])
    elif event['request']['type'] == "SessionEndedRequest":
        return on_session_ended(event['request'], event['session'])


def on_session_started(session_started_request, session):
    """ Called when the session starts """

    print("on_session_started requestId=" + session_started_request['requestId']
          + ", sessionId=" + session['sessionId'])


def on_launch(launch_request, session):
    """ Called when the user launches the skill without specifying what they
    want
    """

    print("on_launch requestId=" + launch_request['requestId'] +
          ", sessionId=" + session['sessionId'])
    # Dispatch to your skill's launch
    return get_welcome_response()


def on_intent(intent_request, session):
    """ Called when the user specifies an intent for this skill """

    print("on_intent requestId=" + intent_request['requestId'] +
          ", sessionId=" + session['sessionId'])

    intent = intent_request['intent']
    intent_name = intent_request['intent']['name']

    # Dispatch to your skill's intent handlers
    if intent_name == "MyPoolInfo":
        return get_poolinfo(intent, session)
    elif intent_name == "MyPoolDev":
        return get_pooldev(intent, session)
    elif intent_name == "MyPoolMode":
        return cmd_poolmode(intent, session)
    elif intent_name == "AMAZON.HelpIntent":
        return get_welcome_response()
    elif intent_name == "AMAZON.CancelIntent":
        return handle_session_end_request()
    elif intent_name == "AMAZON.StopIntent":
        return handle_session_end_request()
    elif intent_name == "AMAZON.FallbackIntent":
        return handle_session_end_request()
    else:
        raise ValueError("Invalid intent")


def on_session_ended(session_ended_request, session):
    """ Called when the user ends the session.
    Is not called when the skill returns should_end_session=true
    """
    print("on_session_ended requestId=" + session_ended_request['requestId'] +
          ", sessionId=" + session['sessionId'])
    # add cleanup logic here

# --------------- Functions that control the skill's behavior ------------------

def get_welcome_response():
    """ If we wanted to initialize the session to have some attributes we could
    add those here
    """

    session_attributes = {}
    card_title = "Welcome"
    speech_output = "Welcome to My Pool. " \
                    "Just ask me What's the pool temperature."
    # If the user either does not reply to the welcome message or says something
    # that is not understood, they will be prompted again with this text.
    reprompt_text = "Ask me, what is the pool temperature. " 
    should_end_session = False
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

def handle_session_end_request():
    card_title = "Session Ended"
    speech_output = "I hope the pool is ok for you. " \
                    "Have a nice day! "
    # Setting this to true ends the session and exits the skill.
    should_end_session = True
    return build_response({}, build_speechlet_response(
        card_title, speech_output, None, should_end_session))

# Run function to get pool info
def get_poolinfo(intent, session):

    card_title = intent['name']
    session_attributes = {}
    should_end_session = False
    
    f = urllib2.urlopen(link) # Get your data
    result = f.read()
    data=json.loads(result)
    temp = data[temp_field]
    mode = data[mode_field]
    spa = data[spa_field]
    pump = data[pump_field]
    tempf = float(temp)

    if 'PoolInfo' in intent['slots']:
        info = intent['slots']['PoolInfo']['value']
        if info=="temperature":
            speech_output = "The temperature is " + temp + " celsius. "
            reprompt_text = ""
            ## Comment on the awesomeness
            if tempf > 24.0:
                comment = " That sounds great for swimming."
            elif tempf > 20.0:
                comment = " It's a little cool but still ok."
            else:
                comment = " It's too cold for swimming. "
            speech_output = speech_output + comment
        elif info=="mode":
            speech_output = "The mode is " + mode + ". "
            reprompt_text = ""
        elif info=="status":
            speech_output = "The temperature is " + temp + " celsius. Mode is " + mode + ". Spa is " + spa + ". Pump is " + pump + ". "
            reprompt_text = ""
        else:
            speech_output = "I'm not sure what you are asking " + \
                            ". Please try again."
            reprompt_text = ""
    else:
        speech_output = "I'm not sure what your asking. " \
                        "Please try again."
        reprompt_text = "I'm not sure what your asking. " \
                        "You can ask me for pool information by saying, " \
                        "what's the temperature, mode or status?"
                        
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

# Run function to get pool dev
def get_pooldev(intent, session):

    card_title = intent['name']
    session_attributes = {}
    should_end_session = False
    
    f = urllib2.urlopen(link) # Get your data
    result = f.read()
    data=json.loads(result)
    temp = data[temp_field]
    mode = data[mode_field]
    spa = data[spa_field]
    pump = data[pump_field]
    tempf = float(temp)

    if 'PoolDev' in intent['slots']:
        info = intent['slots']['PoolDev']['value']
        if info=="spa":
            speech_output = "The spa is " + spa + ". "
            reprompt_text = ""
        elif info=="pump":
            speech_output = "The pump is " + pump + ". "
            reprompt_text = ""
        else:
            speech_output = "I'm not sure what you are asking " + \
                            ". Please try again."
            reprompt_text = ""
    else:
        speech_output = "I'm not sure what your asking. " \
                        "Please try again."
        reprompt_text = "I'm not sure what your asking. " \
                        "You can ask me for device information by saying, " \
                        "what's the spa or pump status? "
                        
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))
		
# Run function to command the pool mode
def cmd_poolmode(intent, session):

    card_title = intent['name']
    session_attributes = {}
    should_end_session = False
    
    if 'PoolMode' in intent['slots']:
        info = intent['slots']['PoolMode']['value']
        if info=="auto":
			f = urllib2.urlopen(linkcmd0) # Write command 0
			result = f.read()
			value = int(result)
			if value <> 0:
				speech_output = " okay"
				reprompt_text = ""
			else:
				speech_output = " The server is busy. Try Again. "
				reprompt_text = ""
        elif info=="on":
            f = urllib2.urlopen(linkcmd1) # Write command 1
            result = f.read()
            value = int(result)
            if value <> 0:
				speech_output = " okay"
				reprompt_text = ""
            else:
				speech_output = " The server is busy. Try Again. "
				reprompt_text = ""
        elif info=="off":
            f = urllib2.urlopen(linkcmd2) # Write command 2
            result = f.read()
            value = int(result)
            if value <> 0:
				speech_output = " okay"
				reprompt_text = ""
            else:
				speech_output = " The server is busy. Try Again. "
				reprompt_text = ""
        else:
            speech_output = "I'm not sure what you are asking " + \
                            ". Please try again."
            reprompt_text = ""
    else:
        speech_output = "I'm not sure what your asking. " \
                        "Please try again."
        reprompt_text = "I'm not sure what your asking. " \
                        "You can ask to change the pool mode by saying, " \
                        "change the pool mode to auto. "
                        
    return build_response(session_attributes, build_speechlet_response(
        card_title, speech_output, reprompt_text, should_end_session))

# --------------- Helpers that build all of the responses ----------------------


def build_speechlet_response(title, output, reprompt_text, should_end_session):
    return {
        'outputSpeech': {
            'type': 'PlainText',
            'text': output
        },
        'card': {
            'type': 'Simple',
            'title': 'SessionSpeechlet - ' + title,
            'content': 'SessionSpeechlet - ' + output
        },
        'reprompt': {
            'outputSpeech': {
                'type': 'PlainText',
                'text': reprompt_text
            }
        },
        'shouldEndSession': should_end_session
    }


def build_response(session_attributes, speechlet_response):
    return {
        'version': '1.0',
        'sessionAttributes': session_attributes,
        'response': speechlet_response
    }