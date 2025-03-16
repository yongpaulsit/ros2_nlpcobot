#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nlpcobot_interfaces.srv import ParseCommand

import spacy

class ParseCommandNode(Node):
    def __init__(self):
        super().__init__('parse_command_node')
        self.get_logger().info("Initializing ParseCommandNode...")
        
        self.service_ = self.create_service(
            ParseCommand,
            'parse_command',
            self.service_callback,
        )
        
        self.nlp = spacy.load("en_core_web_sm")

    def service_callback(self, request, response):
        self.get_logger().info('Parsing Input: "%s"' % request.text)
        try:
            response.action, response.labels = self.extract_action_labels(request.text)
            self.get_logger().info(f'Parsing Result: Action: {response.action}, Labels: {response.labels}')
        except Exception as e:
            self.get_logger().warn(e)
            response = ParseCommand.Response()
        return response

    def extract_action_labels(self, text):
        doc = self.nlp(text.lower())

        action = ''
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
        
def main(args=None):
    rclpy.init(args=args)
    
    node = ParseCommandNode()
    
    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        rclpy.spin(node)
            
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
