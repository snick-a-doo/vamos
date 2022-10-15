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

#include "xml-parser.h"
#include "../geometry/conversions.h"
#include "../geometry/three-vector.h"

#include <algorithm>
#include <cassert>
#include <sstream>
#include <stdexcept>

using namespace Vamos_Geometry;
using namespace Vamos_Media;

namespace Vamos_Media
{
Ac3d get_model(pugi::xml_node node, std::string const& dir)
{
    return Ac3d{dir + get_value(node, "file", std::string()),
                get_value(node, "scale", 1.0),
                get_value(node, "translate", Three_Vector::ZERO),
                get_value(node, "rotate", Three_Vector::ZERO) * deg_to_rad(1.0)};
}
}


class XML_Exception : std::runtime_error
{
public:
    XML_Exception(std::string const& file, int line = 0, std::string message = "")
        : std::runtime_error{file + ':' + std::to_string(line) + ' ' + message}
    {}
};

struct XML_Unterminated
{
    int lines;
    std::string text;
    bool eof;
    char delimiter;
};

//----------------------------------------------------------------------------------------
using Str_Iter = std::string::iterator;

static void get_text_boundries(XML_Tag::Tag type, Str_Iter& text_start, Str_Iter& text_end)
{
    if (type == XML_Tag::Tag::processing || type == XML_Tag::Tag::end)
        ++text_start;
    if (type == XML_Tag::Tag::processing || type == XML_Tag::Tag::empty)
        --text_end;
}

static Vamos_Media::XML_Tag::Tag find_tag_type(std::string const& tag_text)
{
    auto first{tag_text[1]};
    auto last{*(tag_text.rbegin() + 1)};
    if (first == '?' && last == '?')
        return XML_Tag::Tag::processing;
    else if (tag_text.starts_with("<!--") && tag_text.ends_with("-->"))
        return XML_Tag::Tag::comment;
    else if (first == '/')
        return XML_Tag::Tag::end;
    else if (last == '/')
        return XML_Tag::Tag::empty;
    return XML_Tag::Tag::start;
}

std::string find_label(Str_Iter text_start, Str_Iter text_end)
{
    auto label_end{std::find(text_start, text_end, ' ')};
    return std::string(text_start, label_end);
}

XML_Tag::XML_Tag(std::ifstream& stream)
{
    std::string tag_text{read_tag(stream)};
    if (tag_text.front() != '<')
        return;
    if (tag_text.empty())
        return;
    m_type = find_tag_type(tag_text);
    if (m_type == Tag::comment)
        return;

    Str_Iter text_start{tag_text.begin() + 1};
    Str_Iter text_end{tag_text.end() - 1};
    get_text_boundries(m_type, text_start, text_end);
    m_label = find_label(text_start, text_end);
    find_attributes(text_start, text_end);
}

std::string XML_Tag::read_tag(std::ifstream& stream)
{
    /// Get the next character from the stream and update the line count.
    auto next_char = [&](char& ch) {
        stream.get(ch);
        if (ch == '\n')
            ++m_lines;
        return stream.good();
    };

    std::string text(2, '\0');
    char& ch{text.front()};
    // Read any data before the tag.
    while (next_char(ch))
    {
        if (ch == '<')
            break;
        if (!m_data.empty() || (ch != ' ' && ch != '\t' && ch != '\n'))
            m_data.push_back(ch);
    }
    // No tag found.
    if (ch != '<')
        return "";

    auto in_comment{false};
    while (next_char(text.back()))
    {
        if (text.ends_with("!--"))
            in_comment = true;
        else if (text.ends_with("--"))
            in_comment = false;
        if (text.back() == '>' && !in_comment)
            return text;
        if (text.back() == '<' && !in_comment)
            break;
        text.push_back('\0');
    }
    throw XML_Unterminated{m_lines, text, false, '>'};
}

void XML_Tag::find_attributes(Str_Iter text_start, Str_Iter text_end)
{
    auto skip_spaces = [](auto& it, auto& end) {
        while (it != end && (*it == ' ' || *it == '\t' || *it == '\n'))
            ++it;
    };

    auto get_attribute = [](Str_Iter text_start, Str_Iter text_end)
    {
        auto mark{std::find(text_start, text_end, '=')};
        std::string name(text_start, mark);
        mark += 2;
        std::string val(mark, std::find(mark, text_end, '"'));
        return std::pair{name, val};
    };

    text_start += m_label.size();
    if (text_start == text_end)
        return;
    ++text_end;
    while (true)
    {
        skip_spaces(text_start, text_end);
        auto attrib_end{std::find(text_start, text_end, '"')};
        if (attrib_end == text_end)
            return;
        attrib_end = std::find(attrib_end + 1, text_end, '"');
        if (attrib_end == text_end)
            throw XML_Unterminated{m_lines, std::string(text_start, text_end), true, '"'};
        auto [name, value] = get_attribute(text_start, attrib_end);
        m_attributes[name] = value;
        text_start = attrib_end + 1;
    }
}

//----------------------------------------------------------------------------------------
std::string XML_Path::top() const
{
    return m_path.substr(m_path.find_last_of("/") + 1);
}

bool XML_Path::match(std::string pattern) const
{
    // An absolute path must match the whole path.
    // A relative path must match part of the path immediately following a slash.
    return pattern.empty() ? m_path.empty()
        : pattern.front() == '/' ? pattern == m_path
        : m_path.ends_with("/" + pattern);
}

//----------------------------------------------------------------------------------------
void XML_Parser::read(std::string file)
{
    m_file = file;
    m_line = 0;
    mp_stream = std::make_unique<std::ifstream>(file.c_str());
    if (!mp_stream || !*mp_stream)
        error("File not found");
    ++m_line;

    try
    {
        read_document();
    }
    catch (XML_Unterminated& unterm)
    {
        // If the tag ends with \n, almost certainly a '>' was forgotten on the previuos
        // line.
        unterm.lines -= std::count(unterm.text.begin(), unterm.text.end(), '\n');
        unterm.text = std::string(unterm.text.begin(),
                                  std::find(unterm.text.begin(), unterm.text.end(), '\n'));
        std::ostringstream message;
        message << '"' << unterm.delimiter << "\" is missing for \"" << unterm.text << '"';
        m_line = unterm.eof ? -1 : m_line + unterm.lines;
        error(message.str());
    }
    if (m_path.empty())
        return;

    error("Unterminated \"<" + m_path.top() + ">\" tag");
}

void XML_Parser::error(std::string message)
{
    throw XML_Exception(m_file, m_line, message);
}

bool XML_Parser::run_callbacks(const XML_Tag& tag)
{
    switch (tag.get_type())
    {
    case XML_Tag::Tag::none:
        return true;
        break;
    case XML_Tag::Tag::start:
        on_data(tag.get_data());
        on_start_tag(tag);
        break;
    case XML_Tag::Tag::end:
        on_data(tag.get_data());
        on_end_tag(tag);
        break;
    case XML_Tag::Tag::empty:
        on_start_tag(tag);
        on_end_tag(tag);
        break;
    case XML_Tag::Tag::comment:
        break;
    default:
        error('"' + tag.get_label() + "\" is not a recognized tag");
        break;
    }
    return false;
}

void XML_Parser::read_document()
{
    // Check that the file starts with <?xml ...>
    XML_Tag tag{*mp_stream};
    m_line += tag.get_lines();
    if (tag.get_type() != XML_Tag::Tag::processing || tag.get_label() != "xml")
        error("XML declaration is missing");

    /// Put a tag label in the path.
    auto add_tag = [&](const XML_Tag& tag) {
        m_path.push(tag.get_label());
    };
    /// Remove a tag label from the path.
    auto remove_tag = [&](const XML_Tag& tag) {
        if (tag.get_label() != m_path.top())
            error("Expected </" + m_path.top() + "> but found </" + tag.get_label() +'>');
        m_path.drop();
    };

    auto done{false};
    while (!done)
    {
        XML_Tag tag{*mp_stream};
        m_line += tag.get_lines();

        // Match start and end tags.
        if (tag.get_type() == XML_Tag::Tag::start || tag.get_type() == XML_Tag::Tag::empty)
            add_tag(tag);
        done = run_callbacks(tag);
        if (tag.get_type() == XML_Tag::Tag::end || tag.get_type() == XML_Tag::Tag::empty)
            remove_tag(tag);
    }
}
