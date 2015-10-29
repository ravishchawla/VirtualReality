#ifndef CSVREADER_H
#define	CSVREADER_H

#include "CSVReader.h"
#include <iostream>

#endif

/** The default constructor of the CSV reader Class */
CsvReader::CsvReader(const string &path, const char &separator) {
	_file.open(path.c_str(), ifstream::in);
	_separator = separator;
}

int stringToInt(std::string &s)
{
	std::istringstream is(s);
	int result;
	return is >> result ? result : 0;
}

/* Read a plane text file with .ply format */
void CsvReader::readPLY(vector<cv::Point3f> &list_vertex, vector<vector<int> > &list_triangles)
{
	std::string line, tmp_str, n;
	int num_vertex = 0, num_triangles = 0;
	int count = 0;
	bool end_header = false;
	bool end_vertex = false;

	// Read the whole *.ply file
	while (getline(_file, line)) {
		stringstream liness(line);

		// read header
		if (!end_header)
		{
			getline(liness, tmp_str, _separator);
			if (tmp_str == "element")
			{
				getline(liness, tmp_str, _separator);
				getline(liness, n);
				if (tmp_str == "vertex") num_vertex = stringToInt(n);
				if (tmp_str == "face") num_triangles = stringToInt(n);
			}
			if (tmp_str == "end_header") end_header = true;
		}

		// read file content
		else if (end_header)
		{
			// read vertex and add into 'list_vertex'
			if (!end_vertex && count < num_vertex)
			{	
				string x, y, z;
				getline(liness, x, _separator);
				getline(liness, y, _separator);
				getline(liness, z);

				cv::Point3f tmp_p;
				tmp_p.x = (float)stringToInt(x);
				tmp_p.y = (float)stringToInt(y);
				tmp_p.z = (float)stringToInt(z);
				list_vertex.push_back(tmp_p);

				count++;
				if (count == num_vertex)
				{
					count = 0;
					end_vertex = !end_vertex;
				}
			}
			// read faces and add into 'list_triangles'
			else if (end_vertex  && count < num_triangles)
			{
				string num_pts_per_face, id0, id1, id2;
				getline(liness, num_pts_per_face, _separator);
				getline(liness, id0, _separator);
				getline(liness, id1, _separator);
				getline(liness, id2);

				std::vector<int> tmp_triangle(3);
				tmp_triangle[0] = stringToInt(id0);
				tmp_triangle[1] = stringToInt(id1);
				tmp_triangle[2] = stringToInt(id2);
				list_triangles.push_back(tmp_triangle);

				count++;
			}
		}
	}
}

