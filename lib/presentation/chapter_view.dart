import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/presentation/white_button.dart';
import 'package:flutter/material.dart';

import 'aware_text.dart';
import 'second_scroll_view.dart';

class ChapterView extends StatelessWidget {
  final Chapter? chapter;

  ChapterView({@required this.chapter});
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(30.0),
      child: Stack(
        children: [
          Align(
            alignment: Alignment(-1, -0.3),
            child: Text(
              chapter?.chapter ?? '-',
              maxLines: 1,
              softWrap: false,
              overflow: TextOverflow.visible,
              style: TextStyle(color: Colors.white10, fontSize: 250),
            ),
          ),
          Align(
            alignment: Alignment(-1, 0.5),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                AwareText(
                  chapter!.title,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white, fontSize: 30),
                  ),
                ),
                SizedBox(height: 20),
                AwareText(
                  chapter!.description,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white60, fontSize: 14),
                  ),
                ),
                SizedBox(height: 30),
                WhiteButton(
                    onTap: () {
                      Navigator.of(context).push(MaterialPageRoute(
                          builder: (context) => SecondScrollView(chapter: chapter)));
                    },
                    text: 'button_chapter')
              ],
            ),
          ),
        ],
      ),
    );
  }
}
