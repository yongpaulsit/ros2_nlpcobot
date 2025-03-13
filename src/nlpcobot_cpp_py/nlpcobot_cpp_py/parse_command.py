#!/usr/bin/env python3
import spacy

nlp = spacy.load("en_core_web_sm")

def extract_action_object(sentence):
    doc = nlp(sentence.lower())

    action = None
    object_labels = []
    current_object = []

    for token in doc:
        if token.pos_ == "VERB":  # Extract action
            action = token.lemma_
        elif token.pos_ in ["ADJ", "NOUN"]:  # Group adjectives with nouns
            current_object.append(token.text)
        elif current_object:  # If we've collected an object, add it as a whole entity
            object_labels.append(" ".join(current_object))
            current_object = []

    if current_object:  # Catch any remaining object at the end
        object_labels.append(" ".join(current_object))

    return action, object_labels

# Test Cases
sentences = [
    "Pick up the Blue Block for me, please",
    "Move left",
    "Place the red ball on the table",
    "Pick the small green cube",
    "Move forward",
    "Grab the yellow cylinder"
]

for sentence in sentences:
    action, objects = extract_action_object(sentence)
    print(f"Sentence: {sentence}")
    print(f"Action: {action}")
    print(f"text_labels: {objects}")
    print()
