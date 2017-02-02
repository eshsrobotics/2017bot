package org.usfirst.frc.team1759.robot;

import java.util.Arrays;
import java.util.List;

import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.DefaultHandler;

public class XMLParser extends DefaultHandler{
	//String that represents that recently seen start tag.
	private String currentElement;
	private String typeOfCurrentElement;
	private String timeStampOfCurrentElement;
	private static List<String> recognizedElementNames = Arrays.asList("type", "timestamp", "data", "papasdistance", "papasangle", "solutionfound", "solutiontype");
	
	// Set to true after we've read a <data> start tag.
	private boolean readingPapasData = false;
	
	@Override
	/**
	 * Whenever our parser encounters the opening of any XML node in the string we're parsing, this method gets called.
	 * 
	 *  For our part, all we do is set a variable, currentElement, to the last node we saw. 
	 */
	public void startElement(String uri, String localName, String qName, Attributes attributes) throws SAXException{
		String qNameInLowerCase = qName.toLowerCase();
		if(recognizedElementNames.contains(qNameInLowerCase)){
			currentElement = qName;
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
	public void characters(char ch[], int start, int length) throws SAXException{
		if(currentElement == "type") {
			typeOfCurrentElement = new String(ch, start, length);
		}
		else if(currentElement == "timestamp"){
			timeStampOfCurrentElement = new String(ch, start, length);
		}
		else if(currentElement == "data") {
			// This is the XML data that we expect to receive:
			//
			// <message>
			//   <type>camera</type>
			//   <timestamp>Jan 1 1970 10:56 AM PDT</timestamp>
			//   <data>
			//     <PapasDistance>4.0</PapasDistance>
			//     <PapasAngle>5.0</PapasAngle>
			//     <SolutionFound>False</SolutionFound>
			//     <SolutionType>Boiler or Peg</SolutionType>
			//   </data>
			// </message>
			
		}
	}
}
