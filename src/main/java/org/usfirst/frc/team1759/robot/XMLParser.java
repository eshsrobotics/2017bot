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
	private static List<String> recognizedElementNames = Arrays.asList("type", "timestamp", "data");
	
	// Set to true after we've read a <data> start tag.
	private boolean readingPapasData = false;
	
	@Override
	public void startElement(String uri, String localName, String qName, Attributes attributes) throws SAXException{
		String qNameInLowerCase = qName.toLowerCase();
		if(recognizedElementNames.contains(qNameInLowerCase)){
			currentElement = qName;
		}
	}
	@Override
	public void characters(char ch[], int start, int length) throws SAXException{
		if(currentElement == "type"){
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
