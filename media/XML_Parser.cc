//  XML_Parser.cc - an XML file reader
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

#include "XML_Parser.h"

#include <iostream>
#include <algorithm>
#include <sstream>
#include <cassert>

using namespace Vamos_Media;

std::string
remove_leading_space (std::string data_string)
{
  std::string::iterator it;
  for (it = data_string.begin (); it != data_string.end (); it++)
	{
	  if ((*it != ' ') && (*it != '\t') && (*it != '\n'))
		{
		  break;
		}
	}
  return std::string (it, data_string.end ());
}

// Construct the message for exceptions.
std::string 
XML_Exception::message () const
{
  std::ostringstream ost;
  ost << m_file << ':';
  if (m_line == -1)
	{
	  ost << "eof";
	}
  else
	{
	  ost << m_line;
	}
  ost << ' ' << m_message;
  return ost.str ();
}

// * Class XML_Tag
// Read up to and including the next tag.

XML_Tag::XML_Tag (std::ifstream& stream) :
  m_type (NONE),
  m_lines (0)
{
  bool done = read_to_tag_start (stream);
  if (!done)
	{
	  done = read_to_tag_end (stream);
	  if (!done)
		{
		  throw Unterminated_Tag (get_lines (), m_text, true);
		}
	}

  m_data = remove_leading_space (m_data);

  if (m_text.size () == 0) 
	{
	  return;
	}

  m_type = find_tag_type (stream);
  if (m_type != COMMENT)
	{
	  String_Iterator text_start;
	  String_Iterator text_end;
	  get_text_boundries (text_start, text_end);
	  m_label = find_label (text_start, text_end);
	  find_attributes (text_start, text_end);
	}
}

void 
XML_Tag::get_text_boundries (String_Iterator& text_start,
							 String_Iterator& text_end)
{
  text_start = m_text.begin () + 1;
  text_end = m_text.end () - 1;
  switch (m_type)
	{
	case PROCESSING_INSTRUCTION:
	  text_start++;
	  text_end--;
	  break;
	case END:
	  text_start++;
	  break;
	case EMPTY:
	  text_end--;
	  break;
	case START:
	  break;
	default:
	  assert (false);
	}
}

// Read everything up to the next '<'.  Return true if '<' was found.
bool 
XML_Tag::read_to_tag_start (std::ifstream& stream)
{
  char ch;
  while (get_next_char (stream, ch))
	{
	  if (ch == '<')
		{
		  m_text.push_back (ch);
		  return false;
		}
	  m_data.push_back (ch);
	}
  return true;
}

// Read everything up to the next `>'.  Return true if '>' was found.
bool 
XML_Tag::read_to_tag_end (std::ifstream& stream)
{
  bool in_comment = false;
  char current = '\0';
  char old = '\0';
  char older = '\0';
  char oldest = '\0';
  while (get_next_char (stream, current))
	{
	  if ((oldest == '!') && (older == '-') && (old == '-'))
		{
		  in_comment = true;
		}
	  if ((current == '<') && !in_comment)
		{
		  throw Unterminated_Tag (get_lines (), m_text, false);
		}
	  m_text.push_back (current);
	  if (current == '>')
		{
		  return true;
		}
	  oldest = older;
	  older = old;
	  old = current;
	}	
  return false;
}

Vamos_Media::XML_Tag::Tag_Type 
XML_Tag::find_tag_type (std::ifstream& stream)
{
  Tag_Type type;
  const size_t last = m_text.size () - 1;
  if ((m_text [1] == '?') && (m_text [last - 1] == '?'))
	{
	  type = PROCESSING_INSTRUCTION;
	}
  else if ((m_text [1] == '!') && (m_text [2] == '-') && (m_text [3] == '-'))
	{
	  type = COMMENT;
	  eat_comment (stream);
	}
  else if (m_text [1] == '/')
	{
	  type = END;
	}
  else if (m_text [last - 1] == '/')
	{
	  type = EMPTY;
	}
  else
	{
	  type = START;
	}
  return type;
}

std::string 
XML_Tag::find_label (String_Iterator text_start,
					 String_Iterator text_end)
{
  String_Iterator label_end = std::find (text_start, text_end, ' ');
  return std::string (text_start, label_end);
}

// Get the next char from the stream and count newlines.
std::ifstream& 
XML_Tag::get_next_char (std::ifstream& stream, char& ch)
{
  ch = '\0';
  stream.get (ch);
  if (ch == '\n')
	{
	  m_lines++;
	}
  return stream;
}

bool 
XML_Tag::find_comment_end (std::ifstream& stream)
{
  // ...otherwise, we have to read the file to find the "-->"
  // comment-ender.
  char current = '\0';
  char old = '\0';
  char older = '\0';
  while (get_next_char (stream, current))
	{
	  if ((current == '>') && (old == '-') && (older == '-'))
		{
		  return true;
		}
	  older = old;
	  old = current;
	}
  return false;
}

// Throw away the remainder of a comment.
void 
XML_Tag::eat_comment (std::ifstream& stream)
{
  // If there are no '>' within the comment, then we have the whole
  // thing...
  const size_t last = m_text.size () - 1;
  if ((m_text [last - 1] == '-') && (m_text [last - 2] == '-'))
	{
	  return;
	}

  if (!find_comment_end (stream))
	{
	  throw Unterminated_Tag (get_lines (), m_text, true);
	}
}

void 
XML_Tag::skip_spaces (String_Iterator& text_start)
{
  while (*text_start == ' ')
	{
	  text_start++;
	}
}

// Parse the attributes.
void 
XML_Tag::find_attributes (String_Iterator text_start,
						  String_Iterator text_end)
{
  text_start += m_label.size ();
  if (text_start == text_end) return;
  text_end++;
  while (true)
	{
	  skip_spaces (text_start);

	  String_Iterator attrib_end = std::find (text_start, text_end, '"');
	  if (attrib_end == text_end)
		{
		  return;
		}
	  attrib_end = std::find (attrib_end + 1, text_end, '"');

	  if (attrib_end == text_end)
		{
		  throw Unterminated_Attribute (get_lines (), 
										std::string (text_start, text_end), 
										true);
		}

	  m_attributes.push_back (get_attribute (text_start, attrib_end));
	  text_start = attrib_end + 1;
	}
}

Vamos_Media::XML_Tag::Attribute 
XML_Tag::get_attribute (String_Iterator text_start,
						String_Iterator text_end)
{
  String_Iterator mark = std::find (text_start, text_end, '=');
  std::string name (text_start, mark);
  mark += 2;
  std::string val (mark, std::find (mark, text_end, '"'));
  return Attribute (name, val);
}

//** Class XML_Path
std::string 
XML_Path::subpath (size_t n) const
{
  size_t start = m_path.length () - 1;
  std::string rest = m_path;
  for (size_t i = 0; i < n; i++)
    {
      start = rest.find_last_of ("/");
      rest = rest.substr (0, start);
    }
  return m_path.substr (start + 1);
}

// Split a string at the wildcard character (*) and return the parts
// in a vector.  A * in the first (last) position yields an empty
// string as the first (last) element of the vector.
std::vector<std::string> split (std::string in)
{
  std::vector <std::string> out;
  size_t start = 0;
  size_t end = in.size ();
  while ((end = in.find ('*', start)) != std::string::npos)
    {
      out.push_back (in.substr (start, end - start));
      start = end + 1;
    }
  out.push_back (in.substr (start, end - start));
  return out;
}

bool 
XML_Path::match (std::string pattern) const
{
  std::vector <std::string> words = split (pattern);
  assert (words.size () > 0);
  // If no wildcard, must match whole string.
  if (words.size () == 1)
    return m_path == pattern;

  size_t start_index = 0;
  for (std::vector <std::string>::iterator it = words.begin ();
       it != words.end () - 1;
       it++)
    {
      if ((start_index = m_path.find (*it, start_index)) == std::string::npos)
        return false;
      // The first (possibly empty) element must match the beginning
      // of the candidate.
      if ((it == words.begin ()) && (start_index > 0))
        return false;
      start_index += it->size ();
    }

  size_t end_index = start_index;
  start_index = m_path.rfind (*(words.end () - 1));
  if (start_index == std::string::npos)
    return false;
  // The last (possibly empty) element must match the end of the path.
  if (start_index + (words.end () - 1)->size () != m_path.size ())
    return false;
  // The last match must not overlap previous matches.
  if (start_index < end_index)
    return false;

  return true;
}

//** Class XML_Parser

XML_Parser::XML_Parser () :
  mp_stream (0)
{
}


XML_Parser::~XML_Parser ()
{
  delete mp_stream;
}

void 
XML_Parser::read (std::string file)
{
  m_file = file;
  mp_stream = new std::ifstream (file.c_str ());
  if ((mp_stream == 0) || (*mp_stream == 0))
	{
	  throw No_XML_File (m_file);
	}
  m_line = 1;

  try
	{
      read_document ();
    }
  catch (XML_Unterminated& unterminated)
	{
	  handle_unterminated (unterminated);
	}

  if (!m_path.empty ())
	{
	  std::string message = 
		"Unterminated \"<" + m_path.top () + ">\" tag";
	  throw Tag_Mismatch (m_file, -1, message);
	}
}

void 
XML_Parser::error (std::string message)
{
  throw XML_Exception (m_file, m_line, message);
}

// Read the XML declaration.
void 
XML_Parser::check_declaration ()
{
  XML_Tag tag (*mp_stream);
  m_line += tag.get_lines ();
  if ((tag.get_type () != XML_Tag::PROCESSING_INSTRUCTION) 
	  || (tag.get_label () != "xml"))
	{
	  throw No_Declaration (m_file, m_line, "XML declaration is missing");
	}
}

bool 
XML_Parser::run_callbacks (const XML_Tag& tag)
{
  switch (tag.get_type ())
	{
	case XML_Tag::NONE:
	  return true;
	  break;
	case XML_Tag::START:
	  on_data (tag.get_data ());
	  on_start_tag (tag);
	  break;
	case XML_Tag::END:
	  on_data (tag.get_data ());
	  on_end_tag (tag);
	  break;
	case XML_Tag::EMPTY:
	  on_start_tag (tag);
	  on_end_tag (tag);
	  break;
	case XML_Tag::COMMENT:
	  break;
	default:
	  std::string message = 
		'"' + tag.get_text () + "\" is an unrecognized tag";
	  throw Bad_Tag_Type (m_file, m_line, message);
	  break;
	}
  return false;
}

void 
XML_Parser::read_document ()
{
  check_declaration ();

  bool done = false;
  while (!done)
	{
	  XML_Tag tag (*mp_stream);
	  m_line += tag.get_lines ();

	  // Match start and end tags.
	  if ((tag.get_type () == XML_Tag::START)
          || (tag.get_type () == XML_Tag::EMPTY))
		{
		  add_tag (tag);
		}

      done = run_callbacks (tag);

	  if ((tag.get_type () == XML_Tag::END)
          || (tag.get_type () == XML_Tag::EMPTY))
		{
		  remove_tag (tag);
		}
	}
}

void 
XML_Parser::add_tag (const XML_Tag& tag)
{
  m_path.push (tag.get_label ());
}

void 
XML_Parser::remove_tag (const XML_Tag& tag)
{
  if (tag.get_label () != m_path.top ())
	{
	  std::string message = "Expected </" + m_path.top ()
		+ "> but found </" + tag.get_label () + '>';
	  throw Tag_Mismatch (m_file, m_line, message);
	}
  m_path.drop ();
}

void
XML_Parser::handle_unterminated (XML_Unterminated& unterminated)
{
  // If the tag ends with \n, almost certainly a '>' was
  // forgotten on the previuos line.
  unterminated.lines -= std::count (unterminated.text.begin (), 
									unterminated.text.end (), '\n');
  unterminated.text = std::string (unterminated.text.begin (), 
								   std::find (unterminated.text.begin (), 
											  unterminated.text.end (), '\n'));
  std::ostringstream message;
  message << '"' << unterminated.delimiter 
		  << "\" is missing for \""
		  << unterminated.text << '"';

  if (unterminated.eof)
	{
	  m_line = -1;
	}
  else
	{
	  m_line += unterminated.lines;
	}
  throw Tag_Mismatch (m_file, m_line, message.str ());
}
