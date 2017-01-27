/*
 * DatabaseHandler.hpp
 *
 *  Created on: 20/apr/2013
 *      Author: alessandro
 */

#ifndef DATABASEHANDLER_HPP_
#define DATABASEHANDLER_HPP_

#include <map>
#include <list>
#include "Page.hpp"
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <boost/filesystem.hpp>
#include <sqlite3.h>
#include <fstream>

using namespace std;
using namespace boost::filesystem;

/*
 * Singleton DatabaseHandler
 * Database handling class
 */
class DatabaseHandler {
public:
	// 0 is the null terminator for strings: adding this value avoid wrong intepretation of the null terminator
	static const uchar AVOIDING_NULL_TERMINATOR_OFFSET = 1;

	/*
	 * Open a database connection to dbPath
	 */
	static void openDatabase(path dbPath) {
		getInstance()._openDatabase(dbPath);
	}

	/*
	 * Close the previous instaured connection
	 */
	static void closeDatabase() {
		getInstance()._closeDatabase();
	}

	/*
	 * Insert a page with index
	 */
	static void insert(const Page& page, int hash_index) {
		getInstance()._insert(page, hash_index);
	}

	/*
	 * Return a list of pages with the given hash_index
	 */
	static void find(int hash_index, list<Page>& pageList) {
		getInstance()._find(hash_index, pageList);
	}

private:
	sqlite3 *db;

	sqlite3_stmt *insert_stmt;
	string preparedSqlString;

	DatabaseHandler() :
		db(NULL), insert_stmt(NULL) {
	}

	static DatabaseHandler& getInstance() {
		// Guaranteed to be destroyed. Instantiated on first use.
		static DatabaseHandler instance;

		return instance;
	}

	void _openDatabase(path dbPath) {
		char *sErrMsg;

		// Open database
		sqlite3_open(dbPath.string().c_str(), &db);

		// Creating table (if not exists)
		if (sqlite3_exec(
				db,
				"CREATE TABLE IF NOT EXISTS page (id INTEGER PRIMARY KEY, pageId TEXT, pointId INTEGER, invariants TEXT, hash_index INTEGER);",
				NULL, NULL, &sErrMsg)) {
			throw std::logic_error(string("FATAL ERROR: Failed to create page table"));
		}

		// Setting properties
		if (sqlite3_exec(db, "PRAGMA synchronous = OFF", NULL, NULL, &sErrMsg)) {
			throw std::logic_error(string("FATAL ERROR: Can't set property \"PRAGMA synchronous = OFF\""));
		}

		if (sqlite3_exec(db, "PRAGMA journal_mode = MEMORY", NULL, NULL, &sErrMsg)) {
			throw std::logic_error(string("FATAL ERROR: Can't set property \"PRAGMA journal_mode = MEMORY\""));
		}

		// Preparing sql string
		preparedSqlString
				= "INSERT INTO 'page' ('pageId', 'pointId', 'invariants', 'hash_index') VALUES (@PAGE_ID, @POINT_ID, @INVARIANTS, @HASH_INDEX)";
		sqlite3_prepare_v2(db, preparedSqlString.c_str(), preparedSqlString.size(), &insert_stmt, NULL);

		// Begin transaction
		if (sqlite3_exec(db, "BEGIN TRANSACTION", NULL, NULL, &sErrMsg)) {
			throw std::logic_error(string("FATAL ERROR: Can't begin transaction"));
		}
	}

	void _insert(const Page& page, int hash_index) {
		// Creating a string from the list of invariants
		stringstream ssInvariants;
		list<unsigned char>::const_iterator invariantsIt;
		for (invariantsIt = page.invariants.begin(); invariantsIt != page.invariants.end(); invariantsIt++) {
			ssInvariants << uchar(*invariantsIt + AVOIDING_NULL_TERMINATOR_OFFSET);
		}

		sqlite3_bind_text(insert_stmt, 1, page.pageId.c_str(), -1, SQLITE_TRANSIENT);
		sqlite3_bind_int(insert_stmt, 2, page.pointId);
		sqlite3_bind_text(insert_stmt, 3, ssInvariants.str().c_str(), -1, SQLITE_TRANSIENT);
		sqlite3_bind_int(insert_stmt, 4, hash_index);

		sqlite3_step(insert_stmt);

		sqlite3_clear_bindings(insert_stmt);
		sqlite3_reset(insert_stmt);
	}

	void _closeDatabase() {
		char *sErrMsg;

		// Creating index (if not exists)
		if (sqlite3_exec(db, "CREATE INDEX IF NOT EXISTS page_hash_index_idx ON page (hash_index)", NULL, NULL,
				&sErrMsg)) {
			throw std::logic_error(string("FATAL ERROR: Failed to create page table index"));
		}

		// End transaction
		if (sqlite3_exec(db, "END TRANSACTION", NULL, NULL, &sErrMsg)) {
			throw std::logic_error(string("FATAL ERROR: Can't end transaction"));
		}

		sqlite3_finalize(insert_stmt);
		sqlite3_close(db);
	}

	void _find(int hash_index, list<Page>& pageList) {
		// Building the select query
		stringstream ssSelectQuery;
		ssSelectQuery << "SELECT * FROM page WHERE hash_index = " << hash_index;

		// Preparing the query
		sqlite3_stmt *select_stmt;
		sqlite3_prepare_v2(db, ssSelectQuery.str().c_str(), ssSelectQuery.str().size(), &select_stmt, NULL);

		// Fetching
		while (true) {
			int s = sqlite3_step(select_stmt);
			if (s == SQLITE_ROW) {
				Page fetchedPage;

				fetchedPage.pageId = (const char*) sqlite3_column_text(select_stmt, 1);
				fetchedPage.pointId = sqlite3_column_int(select_stmt, 2);
				const unsigned char* invariantsString = sqlite3_column_text(select_stmt, 3);

				while (*invariantsString != '\0') {
					fetchedPage.invariants.push_back(*invariantsString - AVOIDING_NULL_TERMINATOR_OFFSET);
					invariantsString++;
				}

				pageList.push_back(fetchedPage);
			} else if (s == SQLITE_DONE) {
				break;
			} else {
				throw std::logic_error(string("FATAL ERROR: Failed to fetch from database"));
			}
		}
		sqlite3_finalize(select_stmt);
	}

	// Dont forget to declare these two. You want to make sure they
	// are unaccessable otherwise you may accidently get copies of
	// your singleton appearing.
	DatabaseHandler(DatabaseHandler const&); // Don't Implement
	void operator=(DatabaseHandler const&); // Don't implement

	~DatabaseHandler() {
	}
};

#endif /* DATABASEHANDLER_HPP_ */

