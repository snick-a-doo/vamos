//  XML_Parser.h - an XML file reader
//
//	Vamos Automotive Simulator
//  Copyright (C) 2004 Sam Varner
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef _XML_PARSER_H_
#define _XML_PARSER_H_

#include <string>
#include <fstream>
#include <vector>

namespace Vamos_Media
{
  //** Exception Classes
  class XML_Exception
  {
  protected:
	std::string m_file;
	int m_line;
	std::string m_message;
  public:
	XML_Exception (std::string file, int line = 0, std::string message = "") :
	  m_file (file),
	  m_line (line),
	  m_message (message)
	{};
	virtual ~XML_Exception () {};

	virtual std::string message () const;
  };

  class No_XML_File : public XML_Exception
  {
  public:
	No_XML_File (std::string file) : XML_Exception (file) {};
	std::string message () const 
	{ return "Can't find the file \"" + m_file + '"'; }
  };

  class No_Declaration : public XML_Exception
  {
  public:
	No_Declaration (std::string file, int line, std::string message) :
	  XML_Exception (file, line, message) {};
  };

  class Bad_Tag_Type : public XML_Exception
  {
  public:
	Bad_Tag_Type (std::string file, int line, std::string message) :
	  XML_Exception (file, line, message) {};
  };

  class Tag_Mismatch : public XML_Exception
  {
  public:
	Tag_Mismatch (std::string file, int line, std::string message) :
	  XML_Exception (file, line, message) {}
  };

  struct XML_Unterminated
  {
	int lines;
	std::string text;
	bool eof;
	char delimiter;

	XML_Unterminated (int lines_in, 
					  std::string text_in, 
					  bool eof_in,
					  char delimiter_in) : 
	  lines (lines_in),
	  text (text_in),
	  eof (eof_in),
	  delimiter (delimiter_in)
	{};
  };

  struct Unterminated_Tag : public XML_Unterminated
  {
	Unterminated_Tag (int lines_in, std::string text_in, bool eof_in)
	  : XML_Unterminated (lines_in, text_in, eof_in, '>') {};
  };

  struct Unterminated_Attribute : public XML_Unterminated
  {
	Unterminated_Attribute (int lines_in, std::string text_in, bool eof_in)
	  : XML_Unterminated (lines_in, text_in, eof_in, '"') {};
  };



  //** Class XML_Tag
  class XML_Tag
  {
  public:
	enum Tag_Type
	  {
		NONE,
		START,
		END,
		EMPTY,
		PROCESSING_INSTRUCTION,
		COMMENT
	  };

	struct Attribute
	{
	  Attribute (std::string name_in, std::string value_in) 
		: name (name_in), value (value_in) {}
	  std::string name;
	  std::string value;
	};

	typedef std::vector <Attribute> Attribute_List;

  private:
	typedef std::string::iterator String_Iterator;

	Tag_Type m_type;	
	int m_lines;
	std::vector <Attribute> m_attributes;
	std::string m_data;
	std::string m_text;
	std::string m_label;

	// Read everything up to the next '<'.  Return true if '<' was found.
	bool read_to_tag_start (std::ifstream& stream);

	// Read everything up to the next `>'.  Return true if '>' was found.
	bool read_to_tag_end (std::ifstream& stream);

	// Determine the type of the tag.
	Tag_Type find_tag_type (std::ifstream& stream);

	// Determine the tag's label.
	std::string find_label (String_Iterator text_start,
							String_Iterator text_end);

	// Get the next character from the stream.
	std::ifstream& get_next_char (std::ifstream& stream, char& ch);

	// Parse attributes.
	void find_attributes (String_Iterator attr_begin,
						  String_Iterator attr_end);
	
	// Throw out characters inside a comment.
	void eat_comment (std::ifstream& stream);
	
	bool find_comment_end (std::ifstream& stream);
	
	void skip_spaces (String_Iterator& text_start);

	Attribute get_attribute (String_Iterator text_start,
							 String_Iterator text_end);
	void get_text_boundries (String_Iterator& text_start,
							 String_Iterator& text_end);
	
  public:
	XML_Tag (std::ifstream& stream);

	Tag_Type get_type () const { return m_type; }
	int get_lines () const { return m_lines; }
	const Attribute_List& get_attributes () const { return m_attributes; }
	std::string get_data () const { return m_data; }
	std::string get_text () const { return m_text; }
	std::string get_label () const { return m_label; }
  };

  //** XML Tag Path Class
  class XML_Path
  {
    std::string m_path;

  public:
    void push (std::string element) { m_path += '/' + element; }
    void drop () { m_path = m_path.substr (0, m_path.find_last_of ("/")); }
    bool empty () const { return m_path.empty (); }
    bool match (std::string pattern) const;
    std::string path () const { return m_path; }
    std::string subpath (size_t n) const;
    std::string top () const { return subpath (1); }
  };

  //** Parser Class
  class XML_Parser
  {
  public:
	XML_Parser ();
	virtual ~XML_Parser ();

	void read (std::string file);
	void error (std::string message);

    bool match (std::string pattern) const { return m_path.match (pattern); }
    std::string path () const { return m_path.path (); }
    std::string label () const { return m_path.top (); }

	// Event handlers overridden by derived classes.
	virtual void on_start_tag (const XML_Tag& tag) = 0;
	virtual void on_end_tag (const XML_Tag& tag) = 0;
	virtual void on_data (std::string data_string) = 0;

  private:
    std::string m_file;
    std::ifstream* mp_stream;
    int m_line;
    XML_Path m_path;

	void check_declaration ();
	void read_document ();
	bool run_callbacks (const XML_Tag& tag);
	void add_tag (const XML_Tag& tag);
	void remove_tag (const XML_Tag& tag);
	void handle_unterminated (XML_Unterminated& unterminated);
  };
}

#endif // not _XML_PARSER_H_
