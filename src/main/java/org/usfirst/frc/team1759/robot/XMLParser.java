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

import org.omg.CORBA.portable.ApplicationException;
import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.SAXNotRecognizedException;
import org.xml.sax.SAXNotSupportedException;
import org.xml.sax.helpers.DefaultHandler;

/**
 * @author uakotaobi and Ari Berkowicz
 *
 */

public class XMLParser extends DefaultHandler{
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
	
	// We encountered an error during parsing.
	private String errorMessage = "";
	private boolean parseError = false;
	
	// The list of XML elements that a valid PapasVision XML document will contain. 
	final static List<String> recognizedElementNames = Arrays.asList("message", "type", "timestamp", "data", "papasdistance", "papasangle", "solutionfound", "solutiontype");
	
	/**
	 * This is a checklist--any complete XML document will see all of the elements that 
	 * we officially recognize at least once. 
	 */
	final Map<String, Integer> countsForRecognizedElements = new HashMap<String, Integer>();
	
	
	public XMLParser() {
		for (String elementName : recognizedElementNames) { 
			countsForRecognizedElements.put(elementName, 0);
		}	
	}
	
	
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
	  */
	
	public PapasData parse(String xmlDocumentString) throws Exception {
		parseError = false;
		try {
			
			SAXParserFactory factory = SAXParserFactory.newInstance();			
			SAXParser parser = factory.newSAXParser();
			ByteArrayInputStream inputStream = new ByteArrayInputStream(xmlDocumentString.getBytes("UTF-8"));
			parser.parse(inputStream, this);
			
			// Did we encounter any sort of error?
			if (parseError) {
				String exceptionMessage = "PapasVision XML parsing error: \"" + errorMessage + "\"";
				System.err.println(exceptionMessage);
				throw new Exception(exceptionMessage);
			}

			// Did we see all the elements we expected?
			for (String elementName : countsForRecognizedElements.keySet()) {
				if (countsForRecognizedElements.get(elementName) < 1) {
					//throw new Exception(String.format("Expected to see at least one <%s> node in the XML document.", elementName));
				} else {
					//System.err.println(String.format("[debug] Saw %d <%s> elements.", countsForRecognizedElements.get(elementName), elementName));
				}
			}
			
			return papasData;
			
		} catch (SAXException e) {			
//			System.err.println("Caught a SAXException when parsing incoming PapasVision XML document.  The message was: \"" + 
//					e.getMessage() + 
//					"\"\n\nThe XML document was: " + xmlDocumentString);
			throw e;
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
				case "data":
				case "type":
				case "timestamp":
					if (location != Location.TOPLEVEL) {
						errorMessage = String.format("The <%s> node should be directly under the <message> node.", currentElement);
						parseError = true;
						return;
					}
					break;
				case "papasdistance":
				case "papasangle":
				case "solutionfound":
				case "solutiontype":
					if (location != Location.INSIDE_DATA_ELEMENT) {
						errorMessage = "Found unexpected <" + currentElement + "/> node outside of <data/> element.";
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
	 * When we leave the <data></data> element, we will no longer expect <PapasDistance> tags and the like.
	 */
	public void endElement(String uri, String localName, String qName) throws SAXException {
		if (parseError) {
			return;
		}
		
		if (qName.equalsIgnoreCase("data")) {
			location = Location.TOPLEVEL;
	    } 
	}
	
	@Override
	/**
	 * This is called when we read the contents of any XML node.  Based on what our current element was,
	 * this tells us what data we need to capture.
	 * 
	 * For instance, if our XML is "<Foo>Bar</Foo>", then ch[] wioll be { 'B', 'a', 'r' }, 
	 * start will be 0, and length will be 3. 
	 */
	public void characters(char ch[], int start, int length) throws SAXException {
		
		if (parseError) {
			return;
		}
		String content = new String(ch, start, length); 
		
		if (currentElement.equals("type")) {
			
			// Heartbeat and log messages are not acceptable.
			if (!content.equals("camera")) {
				errorMessage = "Unexpected message type \"" + content + "\"; expected \"camera\".";
				parseError = true;
				return;
			} 
			typeOfCurrentElement = content;
			
		}
		else if (currentElement.equals("timestamp")) {
			
			timeStampOfCurrentElement = content;

		}
		else {
			
			// Read the <data/>, which is the most important part of any PapasVision XML message.
			// Then parse it.
			
			String elementNameInLowerCase = currentElement.toLowerCase();
			switch(elementNameInLowerCase) {
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
					errorMessage = "Internal error: unrecognized element <" + currentElement + "> somehow escaped the startElement() filter.";
					parseError = true;
					return;
			}				
		}					
	} // end (void characters())
	
} // end (class XMLParser) 
