/*
 * ezxml - xml parser from eCos
 *
 * Copyright (C) 2004-2006 Aaron Voisine <aaron@voisine.org>
 * Copyright (C) 2005 eCosCentric Ltd
 * Copyright (C) 2010 Robert Schwebel <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#ifndef LINUX_EZXML_H
#define LINUX_EZXML_H

#include <linux/types.h>

#define EZXML_BUFSIZE 1024	/* size of internal memory buffers */
#define EZXML_NAMEM   0x80	/* name is malloced */
#define EZXML_TXTM    0x40	/* txt is malloced */
#define EZXML_DUP     0x20	/* attribute name and value are strduped */

struct ezxml {
	char *name;		/* tag name */
	char **attr;		/* tag attributes { name, value, name, value, ... NULL } */
	char *txt;		/* tag character content, empty string if none */
	size_t off;		/* tag offset from start of parent tag character content */
	struct ezxml *next;	/* next tag with same name in this section at this depth */
	struct ezxml *sibling;	/* next tag with different name in same section and depth */
	struct ezxml *ordered;	/* next tag, same section and depth, in original order */
	struct ezxml *child;	/* head of sub tag list, NULL if none */
	struct ezxml *parent;	/* parent tag, NULL if current tag is root tag */
	short flags;		/* additional information */
};

/*
 * Given a string of xml data and its length, parses it and creates an ezxml
 * structure. For efficiency, modifies the data by adding null terminators
 * and decoding ampersand sequences. If you don't want this, copy the data and
 * pass in the copy. Returns NULL on failure.
 */
struct ezxml* ezxml_parse_str(char *s, size_t len);

/*
 * returns the first child tag (one level deeper) with the given name or NULL
 * if not found
 */
struct ezxml* ezxml_child(struct ezxml *xml, const char *name);

/*
 * returns the next tag of the same name in the same section and depth or NULL
 * if not found
 */
#define ezxml_next(xml) ((xml) ? xml->next : NULL)

/*
 * Returns the Nth tag with the same name in the same section at the same depth
 * or NULL if not found. An index of 0 returns the tag given.
 */
struct ezxml* ezxml_idx(struct ezxml *xml, int idx);

/* returns the name of the given tag */
#define ezxml_name(xml) ((xml) ? xml->name : NULL)

/* returns the given tag's character content or empty string if none */
#define ezxml_txt(xml) ((xml) ? xml->txt : "")

/* returns the value of the requested tag attribute, or NULL if not found */
const char *ezxml_attr(struct ezxml *xml, const char *attr);

/*
 * Traverses the ezxml sturcture to retrieve a specific subtag. Takes a
 * variable length list of tag names and indexes. The argument list must be
 * terminated by either an index of -1 or an empty string tag name. Example:
 * title = ezxml_get(library, "shelf", 0, "book", 2, "title", -1);
 * This retrieves the title of the 3rd book on the 1st shelf of library.
 * Returns NULL if not found.
 */
struct ezxml* ezxml_get(struct ezxml *xml, ...);

/*
 * Converts an ezxml structure back to xml. Returns a string of xml data that
 * must be freed.
 */
char *ezxml_toxml(struct ezxml *xml);

/*
 * returns a NULL terminated array of processing instructions for the given
 * target
 */
const char **ezxml_pi(struct ezxml *xml, const char *target);

/* frees the memory allocated for an ezxml structure */
void ezxml_free(struct ezxml *xml);

/* returns parser error message or empty string if none */
const char *ezxml_error(struct ezxml *xml);

/* returns a new empty ezxml structure with the given root tag name */
struct ezxml* ezxml_new(const char *name);

/* wrapper for ezxml_new() that strdup()s name */
#define ezxml_new_d(name) ezxml_set_flag(ezxml_new(strdup(name)), EZXML_NAMEM)

/*
 * Adds a child tag. off is the offset of the child tag relative to the start
 * of the parent tag's character content. Returns the child tag.
 */
struct ezxml* ezxml_add_child(struct ezxml *xml, const char *name, size_t off);

/* wrapper for ezxml_add_child() that strdup()s name */
#define ezxml_add_child_d(xml, name, off) \
    ezxml_set_flag(ezxml_add_child(xml, strdup(name), off), EZXML_NAMEM)

/* sets the character content for the given tag and returns the tag */
struct ezxml* ezxml_set_txt(struct ezxml *xml, const char *txt);

/* wrapper for ezxml_set_txt() that strdup()s txt */
#define ezxml_set_txt_d(xml, txt) \
    ezxml_set_flag(ezxml_set_txt(xml, strdup(txt)), EZXML_TXTM)

/*
 * Sets the given tag attribute or adds a new attribute if not found. A value
 * of NULL will remove the specified attribute. Returns the tag given.
 */
struct ezxml* ezxml_set_attr(struct ezxml *xml, const char *name, const char *value);

/* Wrapper for ezxml_set_attr() that strdup()s name/value. Value cannot be NULL */
#define ezxml_set_attr_d(xml, name, value) \
    ezxml_set_attr(ezxml_set_flag(xml, EZXML_DUP), strdup(name), strdup(value))

/* sets a flag for the given tag and returns the tag */
struct ezxml* ezxml_set_flag(struct ezxml *xml, short flag);

/* removes a tag along with its subtags without freeing its memory */
struct ezxml* ezxml_cut(struct ezxml *xml);

/* inserts an existing tag into an ezxml structure */
struct ezxml* ezxml_insert(struct ezxml *xml, struct ezxml *dest, size_t off);

/*
 * Moves an existing tag to become a subtag of dest at the given offset from
 * the start of dest's character content. Returns the moved tag.
 */
#define ezxml_move(xml, dest, off) ezxml_insert(ezxml_cut(xml), dest, off)

/* removes a tag along with all its subtags */
#define ezxml_remove(xml) ezxml_free(ezxml_cut(xml))

#endif
