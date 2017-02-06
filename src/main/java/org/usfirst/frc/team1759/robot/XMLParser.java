package org.usfirst.frc.team1759.robot;

import java.io.ByteArrayInputStream;
import java.io.StringReader;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

import javax.xml.parsers.SAXParserFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;

import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.SAXNotRecognizedException;
import org.xml.sax.SAXNotSupportedException;
import org.xml.sax.helpers.DefaultHandler;

/**
 * @author uakotaobi and Ari Berkowicz
 *
 */

public class XMLParser extends DefaultHandler {
        //String that represents that recently seen start tag.
        private String currentElement;
        private String typeOfCurrentElement;
        private String timeStampOfCurrentElement;

        /**
         * Where are we in the XML document right now?
         */
        private enum Location {
            INVALID,            // Haven't seen <message> yet.
            TOPLEVEL,           // Inside <message>, but not inside <data>.
            INSIDE_DATA_ELEMENT // Inside <data>.
        }
        private Location location = Location.INVALID;

        // Stores the private variables that we extract using the SAX event stream.
        private PapasData papasData = new PapasData();

        /**
         * Whenever we encounter an error during parsing, this string is set.
         */
        private String errorMessage = "";

        /**
         * An error during parsing sets this flag, which forces the rest of the parsing to be subsequently skipped.
         */
        private boolean parseError = false;

        /**
         * The list of XML elements that a valid PapasVision XML document will contain.
         */
        final static List<String> recognizedElementNames = Arrays.asList("message", "type", "timestamp", "data", "papasdistance", "papasangle", "solutionfound", "solutiontype");

        /**
         * This is a checklist--any complete XML document will see all of the elements that
         * we officially recognize at least once.
         */
        final Map<String, Integer> countsForRecognizedElements = new HashMap<String, Integer>();

        /**
         * Creates a fresh XMLParser object.
         */
        public XMLParser() { }


        /**
         * The public interface for the XMLparser class.
         *
         * We expect an XML document in the form of a string (for now.)
         *
         * We then extract the PapasVision parameters from this document and return it.
         * This is the XML data that we expect to receive (though not necessarily in
         * this node order):
         *
         * <message>
         *   <type>camera</type>
         *   <timestamp>Jan 1 1970 10:56 AM PDT</timestamp>
         *   <data>
         *     <PapasDistance>4.0</PapasDistance>
         *     <PapasAngle>5.0</PapasAngle>
         *     <SolutionFound>False</SolutionFound>
         *     <SolutionType>Boiler or Peg</SolutionType>
         *   </data>
         * </message>
         * 
         * Yeah, we could probably write some XSD to do this validation for us, but checking
         * the XML programmatically took less time to write.
         */

        public PapasData parse(String xmlDocumentString) throws Exception {

        	// Reset this object. 
            parseError = false;
            for (String elementName : recognizedElementNames) {
                countsForRecognizedElements.put(elementName, 0);
            }

            try {

                SAXParserFactory factory = SAXParserFactory.newInstance();
                SAXParser parser = factory.newSAXParser();
                ByteArrayInputStream inputStream = new ByteArrayInputStream(xmlDocumentString.getBytes("UTF-8"));
                parser.parse(inputStream, this);

                // Did we encounter any sort of error?  Let our caller deal with it.
                if (parseError) {
                    throw new XMLParserException(xmlDocumentString, errorMessage);
                }

                // Did we see all the elements we expected?
                for (String elementName : countsForRecognizedElements.keySet()) {
                	int count = countsForRecognizedElements.get(elementName);
                    if (count != 1) {
                        throw new XMLParserException(xmlDocumentString,
                        		String.format("Expected to see exactly one <%s> node in the XML document, not %d.", elementName, count));
                    }
                }

                return papasData;

            } catch (SAXException e) {
            	
            	// If we had a SAX error, chain it together with our own take on it. 
            	String message = String.format("Caught a SAXException when parsing incoming PapasVision XML document.  The message was: \"%s\".  The XML document was: %s",
            			e.getMessage(),
            			xmlDocumentString);
                throw new XMLParserException(xmlDocumentString, message, e);
            }
        }


        @Override
        /**
         * Whenever our parser encounters the opening of any XML node in the string we're parsing, this method gets called.
         *
         *  For our part, all we do is set a variable, currentElement, to the last node we saw.
         */
        public void startElement(String uri, String localName, String qName, Attributes attributes) throws SAXException {

            if (parseError) {
                return;
            }

            String qNameInLowerCase = qName.toLowerCase();
            if (recognizedElementNames.contains(qNameInLowerCase)) {
                currentElement = qName;

                // This is a recognized element.  Count the number of these we have seen so far.
                //
                // In a saner language, this could be done with
                // "countsForRecognizedElements[qName]++".
                int currentCount = countsForRecognizedElements.get(qName);
                countsForRecognizedElements.put(qName, currentCount + 1);

                // See if the current element is in the expected location.
                switch (qNameInLowerCase) {
                    case "message":
                        if (location != Location.INVALID) {
                            errorMessage = String.format("The <%s> node may only occur as the root element.", qName);
                            parseError = true;
                            return;
                        }
                        break;
                    case "data":
                    case "type":
                    case "timestamp":
                        if (location != Location.TOPLEVEL) {
                            errorMessage = String.format("The <%s> node should be directly under the <message> node.", qName);
                            parseError = true;
                            return;
                        }
                    break;
                    case "papasdistance":
                    case "papasangle":
                    case "solutionfound":
                    case "solutiontype":
                        if (location != Location.INSIDE_DATA_ELEMENT) {
                            errorMessage = String.format("Found unexpected <%s/> node outside of <data/> element.", qName);
                            parseError = true;
                            return;
                        }
                    break;
                }

                // Update our "location" in the document.
                if (currentElement.equals("message")) {
                    location = Location.TOPLEVEL;
                }
                else if (currentElement.equals("data")) {
                    location = Location.INSIDE_DATA_ELEMENT;
                }

            } else {
                errorMessage ="Unrecognized XML element: <" + qName + ">.";
                parseError = true;
            }
        }

        @Override
        /**
         * This event handler gets called once for the end of every XML element in the SAX stream.
         */
        public void endElement(String uri, String localName, String qName) throws SAXException {
            if (parseError) {
                return;
            }

            if (qName.equalsIgnoreCase("data")) {
                location = Location.TOPLEVEL;
            }
            if (qName.equalsIgnoreCase("message")) {
                location = Location.INVALID;
            }
        }

        @Override
        /**
         * This is called when we read the text contents between the start and end tags of any XML node.
         *
         * Based on what our current element was, this tells us what data we need to capture.
         *
         * For instance, if our XML is "<Foo>Bar</Foo>", then ch[] will be { 'B', 'a', 'r' },
         * start will be 0, and length will be 3.
         *
         * NOTE: This function is NOT called for self-closing tags, like <Foo/>.
         */
        public void characters(char ch[], int start, int length) throws SAXException {

            if (parseError) {
                return;
            }
            String content = new String(ch, start, length);
            String elementNameInLowerCase = currentElement.toLowerCase();

            switch (elementNameInLowerCase) {
                case "type":

                    // Heartbeat and log messages are not acceptable.
                    if (!content.equals("camera")) {
                        errorMessage = String.format("Unexpected message type '%s'; expected 'camera'.", content);
                        parseError = true;
                        return;
                    }
                    typeOfCurrentElement = content;
                    break;

                case "timestamp":

                    papasData.timestamp = content;
                    break;

                case "papasdistance":

                    papasData.papasDistanceInInches = Double.parseDouble(content);
                    if (papasData.papasDistanceInInches < 0) {
                        // Not a valid message; this would imply that the camera is behind the target!
                        errorMessage = String.format("The camera is NOT %.3f inches behind the target, you liar.", -papasData.papasDistanceInInches);
                        parseError = true;
                        return;
                    }
                    break;

                case "papasangle":

                    papasData.papasAngleInDegrees = Double.parseDouble(content);
                    break;

                case "solutionfound":

                    papasData.solutionFound = Boolean.parseBoolean(content);
                    break;

                case "solutiontype":

                    if (!content.equals("Peg") &&
                        !content.equals("Boiler")) {

                        errorMessage = String.format("Unrecognized <SolutionType> \"%s\".  Only \"Peg\" and \"Boiler\" are accepted (case-sensitive.)", content);
                        parseError = true;
                        return;
                    }
                    papasData.solutionType = content;
                    break;

                default:

                    // We don't expect to read the contents of anything which is not a recognized element.
                    // Those were rejected early.  Control should never make it here.
                    errorMessage = String.format("Internal error: unrecognized element <%s> somehow escaped the startElement() filter.", currentElement);
                    parseError = true;
                    return;
            }
        } // end (void characters())

} // end (class XMLParser)
