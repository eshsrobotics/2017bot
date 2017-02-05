package org.usfirst.frc.team1759.robot;

import java.io.ByteArrayInputStream;
import java.io.StringReader;
import java.util.Arrays;
import java.util.List;

import javax.xml.parsers.SAXParserFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;

import org.omg.CORBA.portable.ApplicationException;
import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.SAXNotRecognizedException;
import org.xml.sax.SAXNotSupportedException;
import org.xml.sax.helpers.DefaultHandler;

public class XMLParser extends DefaultHandler{
	//String that represents that recently seen start tag.
	private String currentElement;
	private String typeOfCurrentElement;
	private String timeStampOfCurrentElement;
	private boolean insideDataElement = false;
	
	// Stores the private variables that we extract using the SAX event stream.
	private PapasData papasData = new PapasData();
	
	// We encountered an error during parsing.
	private String errorMessage = "";
	private boolean parseError = false;
	
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
			
			return papasData;
			
		} catch (SAXException e) {			
			System.err.println("Caught a SAXException when parsing incoming PapasVision XML document.  The message was: \"" + 
					e.getMessage() + 
					"\"\n\nThe XML document was: " + xmlDocumentString);
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
		
		final List<String> recognizedElementNames = Arrays.asList("type", "timestamp", "data", "papasdistance", "papasangle", "solutionfound", "solutiontype");
				
		String qNameInLowerCase = qName.toLowerCase();
		if (recognizedElementNames.contains(qNameInLowerCase)) {
			currentElement = qName;
			if (currentElement == "data") {
				insideDataElement = true;
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
			insideDataElement = false;
	    }
		
		// TODO: Complain if we don't see all of expected elements. 
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
		
		if(currentElement == "type") {
			typeOfCurrentElement = content;
			
			// Heartbeat and log messages are not acceptable.
			if (typeOfCurrentElement != "camera") {
				errorMessage = "Unexpected message type \"" + typeOfCurrentElement + "\"; expected \"camera\".";
				parseError = true;
				return;
			}
		}
		else if(currentElement == "timestamp"){
			timeStampOfCurrentElement = content;
		}
		else {
			
			// Read the <data/>, which is the most important part of any PapasVision XML message.
			
			String elementNameInLowerCase = currentElement.toLowerCase(); 
			if ((elementNameInLowerCase == "papasdistance" || elementNameInLowerCase == "papasangle" ||
					elementNameInLowerCase == "solutionfound" || elementNameInLowerCase == "solutiontype") &&
				insideDataElement == false) {
				
				// It's <data> *then* <papasDistance>, not the other way around.
				errorMessage = "Found unexpected <" + currentElement + "/> node outside of <data/> element.";
				parseError = true;
				return;
			}
			
			// If control reaches it here, we've confirmed that we're reading one of the
			// <data/> elements in the correct context.  Parse them out.
			if (elementNameInLowerCase == "papasdistance") {
				
				papasData.papasDistanceInInches = Double.parseDouble(content);
				
			} else if (elementNameInLowerCase == "papasangle") {
				
				papasData.papasAngleInDegrees = Double.parseDouble(content);
				
			} else if (elementNameInLowerCase == "solutionfound") {
				
				papasData.solutionFound = Boolean.parseBoolean(content);
				
			} else if (elementNameInLowerCase == "solutiontype") {
				
				papasData.solutionType = content;
				
			} else {

				// We don't expect to read the contents of anything which is not a recognized element.
				// Those were rejected early.  Control should never make it here.
				errorMessage = "Internal error: unrecognized element <" + currentElement + "> somehow escaped the startElement() filter.";
				parseError = true;
				return;
			}				
		}					
	}
}
