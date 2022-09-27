//  Copyright (C) 2004-2022 Sam Varner
//
//  This file is part of Vamos Automotive Simulator.
//
//  Vamos is free software: you can redistribute it and/or modify it under the terms of
//  the GNU General Public License as published by the Free Software Foundation, either
//  version 3 of the License, or (at your option) any later version.
//
//  Vamos is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vamos.
//  If not, see <http://www.gnu.org/licenses/>.

#ifndef VAMOS_MEDIA_XML_PARSER_H_INCLUDED
#define VAMOS_MEDIA_XML_PARSER_H_INCLUDED

#include <fstream>
#include <map>
#include <memory>
#include <string>

namespace Vamos_Media
{
class XML_Tag
{
public:
    /// The types of < > tags expected.
    enum class Tag{none, processing, start, end, empty, comment};
    /// The name/value pairs in the tag.
    using Attributes = std::map<std::string, std::string>;

public:
    /// Read the next tag from the stream and store any data that precedes it.
    XML_Tag(std::ifstream& stream);

    /// @return The tag type.
    Tag get_type() const { return m_type; }
    /// @return The number of lines read so far.
    int get_lines() const { return m_lines; }
    /// @return A map of values to names in the tag.
    Attributes const& get_attributes() const { return m_attributes; }
    /// @return The text that preceded the tag.
    std::string get_data() const { return m_data; }
    /// @return The name in the tag.
    std::string get_label() const { return m_label; }

private:
    using Str_Iter = std::string::iterator;

    /// Read everything up to the next '<' storing in m_data.
    /// @return The text in the tag from < to >.
    std::string read_tag(std::ifstream& stream);
    /// Read attributes.into m_attributes.
    void find_attributes(Str_Iter attr_begin, Str_Iter attr_end);

    Tag m_type{Tag::none}; ///< The kind of tag being read.
    int m_lines{0}; ///< The number of newline character read.
    Attributes m_attributes; ///< Name/value pairs in the tag.
    std::string m_data; ///< For end tags, the text between start and end tags.
    std::string m_text; ///< All the text in the tag, including < and >.
    std::string m_label; ///< The first string after < in a start tag.
};

//----------------------------------------------------------------------------------------
/// A string of slash-separated nested tag names with a stack interface.
class XML_Path
{
public:
    void push(std::string element) { m_path += '/' + element; }
    void drop() { m_path = m_path.substr(0, m_path.find_last_of("/")); }
    bool empty() const { return m_path.empty(); }
    /// True if pattern starts with a / and matches the whole path, or pattern matches a
    /// tail of the path following a /.
    bool match(std::string pattern) const;
    std::string const& path() const { return m_path; }
    /// The current innermost tag.
    std::string top() const;

private:
    std::string m_path;
};

//----------------------------------------------------------------------------------------
class XML_Parser
{
public:
    virtual ~XML_Parser() = default;
    /// Read the XML file and call the event handlers according to the content.
    void read(std::string file);

protected:
    /// Convenience method to raise an error from a derived parser.
    void error(std::string message);
    /// Look for a string match in a tag path. If pattern starts with / compare to the
    /// whole path, else compare to any tail starting after a /.
    bool match(std::string pattern) const { return m_path.match(pattern); }

    /// Event handlers overridden by derived classes.
    /// @{
    /// Called when a start tag is found.
    virtual void on_start_tag(XML_Tag const& tag) = 0;
    /// Called when an end tag is found.
    virtual void on_end_tag(XML_Tag const& tag) = 0;
    /// Called with the data between the tags.
    virtual void on_data(std::string const& data) = 0;
    /// @}

private:
    void read_document();
    /// Call the event handlers when a tag is found.
    bool run_callbacks(XML_Tag const& tag);

    std::string m_file; ///< The XML file name.
    std::unique_ptr<std::ifstream> mp_stream; ///< The stream for reading the file.
    int m_line; ///< The current line number.
    XML_Path m_path; ///< The current sequence of tag labels.
};
} // namespace Vamos_Media

#endif // VAMOS_MEDIA_XML_PARSER_H_INCLUDED
