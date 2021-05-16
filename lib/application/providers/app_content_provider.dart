import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/domain/entity/contact.dart';
import 'package:dorvan/domain/entity/intro.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:flutter/material.dart';

final appContentProvider = Provider<List<PageContent>>((ref) {
  return [
    Intro(
      "chapter_intro_title",
      "chapter_intro_description",
      '00',
    ),
    Picture(AssetImage('assets/images/dorvan.jpg'), 'Vans', 'salut'),
    // Entrepreneur
    Chapter('chapter_entrepreneur_title', "chapter_entrepreneur_description",
        '01', [
      Article('article_entrepreneur_title', 'article_entrepreneur_description',
          '01'),
      Article('article_parlezmoi_title', 'article_parlezmoi_description', '01'),
      Article(
          'article_tirerprofit_title', 'article_tirerprofit_description', '01'),
      Article(
          'article_boutenbout_title', 'article_boutenbout_description', '01'),
    ]),
    Picture(AssetImage('assets/images/logo_presentation.jpg'), 'Vans', 'salut'),
    // Architect
    Chapter('chapter_architect_title', "chapter_architect_description", '02', [
      Article('article_architect_title', 'article_architect_description', '02'),
      Article('article_nature_title', 'article_nature_description', '02'),
      Article('article_pwa_title', 'article_pwa_description', '02'),
    ]),
    Picture(AssetImage('assets/images/02_iphone12.jpg'), 'Vans', 'salut'),
    Chapter('chapter_designer_title', "chapter_designer_description", '03', [
      Article('article_designer_title', 'article_designer_description', '03'),
      Article('article_artetweb_title', 'article_artetweb_description', '03'),
      Article('article_premiereimpression_title',
          'article_premiereimpression_description', '03'),
      Article(
          'article_matechnique_title', 'article_matechnique_description', '03'),
    ]),
    Chapter(
        'chapitre_developer_title', "chapitre_developer_description", '04', [
      Article('article_developer_title', 'article_developer_description', '04'),
      Article(
          'article_technologie_title', 'article_technologie_description', '04'),
      Article('article_language_title', 'article_language_description', '04'),
      Article(
          'article_monsiteweb_title', 'article_monsiteweb_description', '04'),
    ]),
    Contact('contact_title'),
  ];
});
