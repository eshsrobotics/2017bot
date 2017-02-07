/**
 * 
 */
package org.usfirst.frc.team1759.robot;

/**
 * @author uakotaobi
 *
 * An exception exclusively thrown by the {@link org.usfirst.frc.team1759.robot.XMLParser}.
 * It stores the XML document that caused the exception, too.
 */
public class XMLParserException extends Exception {
	private static final long serialVersionUID = -2564616137205975249L;
	private String xmlDocumentString;
	public XMLParserException(String xmlDocumentString, String message) {
		super(message);
		this.xmlDocumentString = xmlDocumentString;
	}
	public XMLParserException(String xmlDocumentString, String message, Throwable cause) {
		super(message, cause);
		this.xmlDocumentString = xmlDocumentString;
	}	
	public String getXmlDocumentString() {
		return xmlDocumentString;
	}
}
